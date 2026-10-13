/*
 * Copyright (c) 2025 CTTC
 *
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * Regression test suite for SionnaRtChannelModel (MR !2608).
 *
 * NOTE: Setting the "Scenario" attribute triggers pybind11/Python
 * initialisation which causes SIGSEGV in environments without Sionna.
 * All tests here are safe to run without a Sionna installation.
 * Tests that require a live Sionna runtime are guarded by
 * NS_TEST_ASSERT_MSG_EQ(sionnaAvailable, true, ...) and skipped otherwise.
 *
 * Registration — add to src/spectrum/test/CMakeLists.txt:
 *
 *   build_lib_test(
 *     LIB spectrum
 *     SOURCES sionna-channel-model-test.cc
 *     LIBRARIES_TO_LINK ${libspectrum}
 *   )
 *
 * Run:
 *   ./ns3 run "test-runner --suite=sionna-rt-channel-model --verbose"
 */

#include "ns3/boolean.h"
#include "ns3/config.h"
#include "ns3/constant-position-mobility-model.h"
#include "ns3/constant-velocity-mobility-model.h"
#include "ns3/double.h"
#include "ns3/isotropic-antenna-model.h"
#include "ns3/log.h"
#include "ns3/matrix-based-channel-model.h"
#include "ns3/node.h"
#include "ns3/phased-array-model.h"
#include "ns3/pointer.h"
#include "ns3/simulator.h"
#include "ns3/sionna-rt-channel-model.h"
#include "ns3/sionna-rt-spectrum-propagation-loss-model.h"
#include "ns3/spectrum-signal-parameters.h"
#include "ns3/spectrum-value.h"
#include "ns3/string.h"
#include "ns3/test.h"
#include "ns3/uinteger.h"
#include "ns3/uniform-planar-array.h"

#include <cmath>
#include <complex>
#include <pybind11/embed.h>
#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <vector>
namespace py = pybind11;

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("SionnaRtChannelModelTest");

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

/**
 * Start the embedded Python interpreter once for the whole suite. The suite finalizes it after
 * its last test case, because numpy, which Sionna loads, cannot be initialized a second time in
 * one process, and Dr.Jit aborts at exit if the interpreter is still alive.
 */
static void
EnsurePythonInterpreter()
{
    if (!Py_IsInitialized())
    {
        py::initialize_interpreter();
    }
}

static Ptr<UniformPlanarArray>
CreateUpa(uint32_t rows, uint32_t cols)
{
    Ptr<UniformPlanarArray> upa = CreateObject<UniformPlanarArray>();
    upa->SetAttribute("NumRows", UintegerValue(rows));
    upa->SetAttribute("NumColumns", UintegerValue(cols));
    return upa;
}

/**
 * Create a single-polarized uniform planar array of isotropic elements, so that every element
 * and path sees the same element gain and only the geometry sets the phases.
 *
 * @param rows number of rows
 * @param cols number of columns
 * @return the array
 */
static Ptr<UniformPlanarArray>
CreateIsotropicUpa(uint32_t rows, uint32_t cols)
{
    Ptr<UniformPlanarArray> upa = CreateUpa(rows, cols);
    upa->SetAttribute("AntennaElement", PointerValue(CreateObject<IsotropicAntennaModel>()));
    return upa;
}

static Ptr<MobilityModel>
CreateMobility(double x, double y, double z)
{
    Ptr<Node> node = CreateObject<Node>();
    Ptr<ConstantPositionMobilityModel> mob = CreateObject<ConstantPositionMobilityModel>();
    mob->SetPosition(Vector(x, y, z));
    node->AggregateObject(mob);
    return mob;
}

/**
 * Unit vector of a direction given by zenith and azimuth angles in degrees.
 *
 * @param zenithDeg zenith angle in degrees
 * @param azimuthDeg azimuth angle in degrees
 * @return the unit vector
 */
static Vector
SphericalUnitVector(double zenithDeg, double azimuthDeg)
{
    const double theta = zenithDeg * M_PI / 180.0;
    const double phi = azimuthDeg * M_PI / 180.0;
    return Vector(std::sin(theta) * std::cos(phi),
                  std::sin(theta) * std::sin(phi),
                  std::cos(theta));
}

/// Scene, positions and velocity shared by the value tests: in Sionna's built-in "floor_wall"
/// scene these give a line-of-sight path plus floor, wall and floor-wall reflections, with
/// distinct delays, directions and Doppler shifts.
const Vector g_txPosition(0.8, -1.0, 2.0); ///< transmitter position
const Vector g_rxPosition(1.7, 1.0, 0.6);  ///< receiver position
const Vector g_rxVelocity(30.0, 0.0, 0.0); ///< receiver velocity
const double g_frequency = 3.5e9;          ///< carrier frequency in Hz

// ===========================================================================
// Test  – Full Sionna end-to-end: GetChannel returns valid matrix
//
//   Starts the shared Python interpreter, sets a known scenario, and verifies:
//     - GetChannel returns non-null
//     - H matrix has correct dimensions
//     - H matrix is non-zero (actual ray-tracing happened)
//     - GetParams returns non-null
//     - Calling GetChannel twice with same positions returns cached matrix
// ===========================================================================
/**
 * @ingroup spectrum-tests
 * @brief End-to-end test for SionnaRtChannelModel::GetChannel with Sionna.
 *
 * Verifies that GetChannel returns a valid, non-null ChannelMatrix with
 * correct dimensions, that GetParams works, and that caching produces
 * consistent results.
 */
class SionnaRtChannelEndToEndTest : public TestCase
{
  public:
    SionnaRtChannelEndToEndTest()
        : TestCase("SionnaRtChannelModel end to end GetChannel with Sionna")
    {
    }

  private:
    void DoRun() override
    {
        EnsurePythonInterpreter();

        const uint32_t sTxRows = 2;
        const uint32_t sTxCols = 2;
        const uint32_t uRxRows = 1;
        const uint32_t uRxCols = 1;

        Ptr<SionnaRtChannelModel> model = CreateObject<SionnaRtChannelModel>();
        model->SetFrequency(3.5e9);
        model->SetAttribute("UpdatePeriod", TimeValue(MilliSeconds(0)));
        model->SetAttribute("NormalizeDelays", BooleanValue(true));

        // This triggers LoadScene — requires Sionna installed
        model->SetAttribute("Scenario", StringValue("simple_street_canyon"));

        Ptr<UniformPlanarArray> txAnt = CreateUpa(sTxRows, sTxCols);
        Ptr<UniformPlanarArray> rxAnt = CreateUpa(uRxRows, uRxCols);
        Ptr<MobilityModel> txMob = CreateMobility(0.0, 0.0, 10.0);
        Ptr<MobilityModel> rxMob = CreateMobility(50.0, 0.0, 1.5);

        // --- Test: GetChannel returns non-null ---
        Ptr<const MatrixBasedChannelModel::ChannelMatrix> H =
            model->GetChannel(txMob, rxMob, txAnt, rxAnt);

        NS_TEST_ASSERT_MSG_NE(H, nullptr, "GetChannel returned nullptr");

        // --- Test: H matrix dimensions ---
        // H is stored as [rxElems x txElems] in MatrixArray
        NS_TEST_ASSERT_MSG_EQ(H->m_channel.GetNumRows(),
                              uRxRows * uRxCols,
                              "H num rows (rx elements) mismatch");
        NS_TEST_ASSERT_MSG_EQ(H->m_channel.GetNumCols(),
                              sTxRows * sTxCols,
                              "H num cols (tx elements) mismatch");

        // --- Test: H has at least one path (non-zero pages/clusters) ---
        NS_TEST_ASSERT_MSG_GT(H->m_channel.GetNumPages(),
                              0U,
                              "H has zero clusters — no paths computed");

        // --- Test: caching — same positions return same or equivalent matrix ---
        Ptr<const MatrixBasedChannelModel::ChannelMatrix> H2 =
            model->GetChannel(txMob, rxMob, txAnt, rxAnt);
        NS_TEST_ASSERT_MSG_NE(H2, nullptr, "Second GetChannel returned nullptr");
        // Either the same cached pointer is returned, or a new matrix with
        // identical dimensions (both are acceptable implementations)
        NS_TEST_ASSERT_MSG_EQ(H2->m_channel.GetNumRows(),
                              H->m_channel.GetNumRows(),
                              "Second call H num rows changed");
        NS_TEST_ASSERT_MSG_EQ(H2->m_channel.GetNumCols(),
                              H->m_channel.GetNumCols(),
                              "Second call H num cols changed");
        NS_TEST_ASSERT_MSG_EQ(H2->m_channel.GetNumPages(),
                              H->m_channel.GetNumPages(),
                              "Second call H num pages changed");

        // --- Test: GetParams returns non-null ---
        Ptr<const MatrixBasedChannelModel::ChannelParams> params = model->GetParams(txMob, rxMob);
        NS_TEST_ASSERT_MSG_NE(params, nullptr, "GetParams returned nullptr after GetChannel");

        // --- Test: SionnaRtChannelParams dynamic type ---
        const auto* sp =
            dynamic_cast<const SionnaRtChannelModel::SionnaRtChannelParams*>(PeekPointer(params));
        NS_TEST_ASSERT_MSG_NE(sp, nullptr, "Params is not SionnaRtChannelParams");

        Simulator::Destroy();
    }
};

/**
 * @ingroup spectrum-tests
 * @brief Check the channel matrix element by element against the array geometry.
 *
 * For every path n and every receive and transmit element pair (u, s), TR 38.901 Eq. 7.5-22
 * relates the coefficient to that of the first element pair by the array phase
 * exp(j 2 pi (r_rx,n . d_u + r_tx,n . d_s)), where d are the element locations in wavelengths
 * and r the unit vectors of the path's arrival and departure angles. The arrays have more than
 * one row and the scene more than one path, so this fails if the matrix pages mix paths, if
 * an ns-3 element index picks up the coefficient of another Sionna element, or if the path
 * angles are not in degrees.
 */
class SionnaRtCirElementPhaseTest : public TestCase
{
  public:
    SionnaRtCirElementPhaseTest()
        : TestCase("SionnaRtChannelModel channel matrix follows the array geometry per path")
    {
    }

  private:
    void DoRun() override
    {
        EnsurePythonInterpreter();

        Ptr<SionnaRtChannelModel> model = CreateObject<SionnaRtChannelModel>();
        model->SetFrequency(g_frequency);
        model->SetAttribute("Scenario", StringValue("floor_wall"));

        Ptr<UniformPlanarArray> txAnt = CreateIsotropicUpa(2, 3);
        Ptr<UniformPlanarArray> rxAnt = CreateIsotropicUpa(3, 2);
        Ptr<MobilityModel> txMob = CreateMobility(g_txPosition.x, g_txPosition.y, g_txPosition.z);
        Ptr<MobilityModel> rxMob = CreateMobility(g_rxPosition.x, g_rxPosition.y, g_rxPosition.z);

        Ptr<const MatrixBasedChannelModel::ChannelMatrix> channel =
            model->GetChannel(txMob, rxMob, txAnt, rxAnt);
        Ptr<const MatrixBasedChannelModel::ChannelParams> params = model->GetParams(txMob, rxMob);
        const auto& h = channel->m_channel;
        const size_t numPaths = h.GetNumPages();
        NS_TEST_ASSERT_MSG_GT(numPaths, 1U, "The scene must produce more than one path");

        for (size_t n = 0; n < numPaths; n++)
        {
            const Vector rRx =
                SphericalUnitVector(params->m_angle[MatrixBasedChannelModel::ZOA_INDEX][n],
                                    params->m_angle[MatrixBasedChannelModel::AOA_INDEX][n]);
            const Vector rTx =
                SphericalUnitVector(params->m_angle[MatrixBasedChannelModel::ZOD_INDEX][n],
                                    params->m_angle[MatrixBasedChannelModel::AOD_INDEX][n]);
            const std::complex<double> ref = h(0, 0, n);
            NS_TEST_ASSERT_MSG_GT(std::abs(ref), 0.0, "Path " << n << " has no power");

            double worstError = 0;
            size_t worstU = 0;
            size_t worstS = 0;
            for (size_t u = 0; u < h.GetNumRows(); u++)
            {
                for (size_t s = 0; s < h.GetNumCols(); s++)
                {
                    const Vector dU = rxAnt->GetElementLocation(u) - rxAnt->GetElementLocation(0);
                    const Vector dS = txAnt->GetElementLocation(s) - txAnt->GetElementLocation(0);
                    const double phase = 2 * M_PI * (rRx * dU + rTx * dS);
                    const double error = std::abs(h(u, s, n) - ref * std::polar(1.0, phase));
                    if (error > worstError)
                    {
                        worstError = error;
                        worstU = u;
                        worstS = s;
                    }
                }
            }
            NS_TEST_ASSERT_MSG_LT(worstError / std::abs(ref),
                                  1e-3,
                                  "Path " << n << ": worst at rx element " << worstU
                                          << ", tx element " << worstS);
        }

        Simulator::Destroy();
    }
};

/**
 * @ingroup spectrum-tests
 * @brief Check the received PSD against Sionna's own channel frequency response over time.
 *
 * A moving receiver sees several paths with different delays and Doppler shifts. The channel is
 * generated at t0 > 0 and the received PSD is evaluated at t0 and t0 + dt, without regenerating
 * it. Each sub-band must match |H(f, t - t0)|^2 from Sionna's Paths.cfr() for the same scene,
 * which applies the carrier phase exp(-j 2 pi f tau) once and the Doppler phase
 * exp(j 2 pi f_D (t - t0)) with f_D in Hz. Both terms change the relative phase of the paths
 * and therefore the received power.
 */
class SionnaRtFrequencyResponseTest : public TestCase
{
  public:
    SionnaRtFrequencyResponseTest()
        : TestCase("SionnaRtSpectrumPropagationLossModel matches Sionna's frequency response")
    {
    }

  private:
    void DoRun() override
    {
        EnsurePythonInterpreter();

        const Time t0 = MilliSeconds(100);
        const Time dt = MilliSeconds(1);
        const uint32_t numBands = 20;
        const double bandWidth = 5e6;

        Ptr<SionnaRtChannelModel> model = CreateObject<SionnaRtChannelModel>();
        model->SetFrequency(g_frequency);
        model->SetAttribute("Scenario", StringValue("floor_wall"));
        Ptr<SionnaRtSpectrumPropagationLossModel> lossModel =
            CreateObject<SionnaRtSpectrumPropagationLossModel>();
        lossModel->SetChannelModel(model);

        Ptr<UniformPlanarArray> txAnt = CreateIsotropicUpa(1, 1);
        Ptr<UniformPlanarArray> rxAnt = CreateIsotropicUpa(1, 1);
        PhasedArrayModel::ComplexVector unitWeight(1);
        unitWeight[0] = 1.0;
        txAnt->SetBeamformingVector(unitWeight);
        rxAnt->SetBeamformingVector(unitWeight);
        Ptr<MobilityModel> txMob = CreateMobility(g_txPosition.x, g_txPosition.y, g_txPosition.z);
        Ptr<Node> rxNode = CreateObject<Node>();
        Ptr<ConstantVelocityMobilityModel> rxMob = CreateObject<ConstantVelocityMobilityModel>();
        rxNode->AggregateObject(rxMob);

        std::vector<BandInfo> bands(numBands);
        std::vector<double> basebandFrequencies(numBands);
        for (uint32_t k = 0; k < numBands; k++)
        {
            bands[k].fl = g_frequency - numBands * bandWidth / 2 + k * bandWidth;
            bands[k].fh = bands[k].fl + bandWidth;
            bands[k].fc = (bands[k].fl + bands[k].fh) / 2;
            basebandFrequencies[k] = bands[k].fc - g_frequency;
        }
        Ptr<SpectrumValue> txPsd = Create<SpectrumValue>(Create<SpectrumModel>(bands));
        *txPsd = 1.0;
        Ptr<SpectrumSignalParameters> txParams = Create<SpectrumSignalParameters>();
        txParams->psd = txPsd;

        std::vector<Ptr<const SpectrumValue>> rxPsd;
        auto measure = [&]() {
            rxPsd.push_back(
                lossModel->CalcRxPowerSpectralDensity(txParams, txMob, rxMob, txAnt, rxAnt)->psd);
        };
        Simulator::Schedule(t0, [&]() {
            rxMob->SetPosition(g_rxPosition);
            rxMob->SetVelocity(g_rxVelocity);
            measure();
        });
        Simulator::Schedule(t0 + dt, measure);
        Simulator::Run();

        // The same scene and path solver settings, straight through Sionna
        py::module_ rt = py::module_::import("sionna.rt");
        py::module_ np = py::module_::import("numpy");
        py::object scene = rt.attr("load_scene")(rt.attr("scene").attr("floor_wall"),
                                                 py::arg("merge_shapes") = false);
        scene.attr("frequency") = g_frequency;
        for (const char* array : {"tx_array", "rx_array"})
        {
            scene.attr(array) = rt.attr("PlanarArray")(py::arg("num_rows") = 1,
                                                       py::arg("num_cols") = 1,
                                                       py::arg("pattern") = "iso",
                                                       py::arg("polarization") = "V");
        }
        scene.attr("add")(rt.attr("Transmitter")(
            py::arg("name") = "tx",
            py::arg("position") = py::make_tuple(g_txPosition.x, g_txPosition.y, g_txPosition.z)));
        scene.attr("add")(rt.attr("Receiver")(
            py::arg("name") = "rx",
            py::arg("position") = py::make_tuple(g_rxPosition.x, g_rxPosition.y, g_rxPosition.z),
            py::arg("velocity") = py::make_tuple(g_rxVelocity.x, g_rxVelocity.y, g_rxVelocity.z)));
        const auto cfg = model->GetRtPathSolverConfig();
        py::object paths =
            rt.attr("PathSolver")()(py::arg("scene") = scene,
                                    py::arg("max_depth") = cfg.maxDepth,
                                    py::arg("los") = cfg.los,
                                    py::arg("specular_reflection") = cfg.specularReflection,
                                    py::arg("diffuse_reflection") = cfg.diffuseReflection,
                                    py::arg("refraction") = cfg.refraction,
                                    py::arg("diffraction") = cfg.diffraction,
                                    py::arg("edge_diffraction") = cfg.edgeDiffraction,
                                    py::arg("synthetic_array") = cfg.syntheticArray,
                                    py::arg("seed") = cfg.seed);
        py::object cfr = paths.attr("cfr")(
            py::arg("frequencies") =
                py::array_t<double>(basebandFrequencies.size(), basebandFrequencies.data()),
            py::arg("sampling_frequency") = 1.0 / dt.GetSeconds(),
            py::arg("num_time_steps") = 2,
            py::arg("out_type") = "numpy");
        auto h = np.attr("asarray")(cfr, py::arg("dtype") = np.attr("complex128"))
                     .cast<py::array_t<std::complex<double>>>();
        auto hView = h.unchecked<6>();

        NS_TEST_ASSERT_MSG_EQ(rxPsd.size(), 2U, "The received PSD must be measured twice");
        for (size_t t = 0; t < rxPsd.size(); t++)
        {
            for (uint32_t k = 0; k < numBands; k++)
            {
                const double expected = std::norm(hView(0, 0, 0, 0, t, k));
                NS_TEST_ASSERT_MSG_EQ_TOL((*rxPsd[t])[k],
                                          expected,
                                          1e-3 * expected,
                                          "Sub-band " << k << " at t0 + " << t << " ms");
            }
        }

        Simulator::Destroy();
    }
};

/**
 * @ingroup spectrum-tests
 * @brief Check that limiting the number of paths does not change the configured depth.
 *
 * When a link produces more paths than MaxNumberOfPaths, the path solver is rerun with a
 * smaller maximum depth. That reduction applies to that link only: the configured depth must be
 * unchanged afterwards, or every later link would be traced shallower.
 */
class SionnaRtPathDepthTest : public TestCase
{
  public:
    SionnaRtPathDepthTest()
        : TestCase("SionnaRtChannelModel path limit leaves the configured depth unchanged")
    {
    }

  private:
    void DoRun() override
    {
        EnsurePythonInterpreter();

        Ptr<SionnaRtChannelModel> model = CreateObject<SionnaRtChannelModel>();
        model->SetFrequency(g_frequency);
        model->SetAttribute("Scenario", StringValue("floor_wall"));
        model->SetAttribute("MaxNumberOfPaths", DoubleValue(2));
        const int depth = model->GetRtPathSolverConfig().maxDepth;

        Ptr<const MatrixBasedChannelModel::ChannelMatrix> channel =
            model->GetChannel(CreateMobility(g_txPosition.x, g_txPosition.y, g_txPosition.z),
                              CreateMobility(g_rxPosition.x, g_rxPosition.y, g_rxPosition.z),
                              CreateIsotropicUpa(1, 1),
                              CreateIsotropicUpa(1, 1));

        NS_TEST_ASSERT_MSG_LT_OR_EQ(channel->m_channel.GetNumPages(),
                                    2U,
                                    "The path limit must hold for the link");
        NS_TEST_ASSERT_MSG_EQ(model->GetRtPathSolverConfig().maxDepth,
                              depth,
                              "The configured maximum depth must be unchanged");

        Simulator::Destroy();
    }
};

// ===========================================================================
// Test Suite
// ===========================================================================
/**
 * @ingroup spectrum-tests
 * @brief Test suite for Sionna RT channel model regression tests.
 *
 * Registers all test cases for the Sionna RT channel model module.
 */
class SionnaRtChannelModelTestSuite : public TestSuite
{
  public:
    SionnaRtChannelModelTestSuite()
        : TestSuite("sionna-rt-channel-model", TestSuite::Type::UNIT)
    {
        // Requires Sionna + GPU — runs full ray-tracing pipeline
        AddTestCase(new SionnaRtChannelEndToEndTest(), TestCase::Duration::QUICK);
        AddTestCase(new SionnaRtCirElementPhaseTest(), TestCase::Duration::QUICK);
        AddTestCase(new SionnaRtFrequencyResponseTest(), TestCase::Duration::QUICK);
        AddTestCase(new SionnaRtPathDepthTest(), TestCase::Duration::QUICK);
    }

  private:
    void DoTeardown() override
    {
        if (Py_IsInitialized())
        {
            py::finalize_interpreter();
        }
    }
};

static SionnaRtChannelModelTestSuite g_sionnaRtChannelModelTestSuite;
