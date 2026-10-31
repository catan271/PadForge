using System;
using PadForge.Common.Input;
using PadForge.Engine;
using PadForge.Engine.Common.Mapping;
using PadForge.Engine.Data;
using Xunit;

namespace PadForge.Tests;

[Collection("SettingsManagerStatics")]
public class GyroStickDirectionTests : IDisposable
{
    private const float Deg = MathF.PI / 180f;
    private readonly string _device = Guid.NewGuid().ToString();
    private readonly Func<string, int, SourceCoercion.GyroTuning> _oldTuning = SourceCoercion.GyroTuningProvider;
    private readonly Func<string, int, (float, float, float)> _oldBias = SourceCoercion.GyroBiasProvider;
    private readonly Func<string, int, (float, float, float)> _oldAuxBias = SourceCoercion.GyroAuxBiasProvider;
    private readonly Func<int, bool> _oldEngage = SourceCoercion.AimEngageStateProvider;
    private readonly Func<float> _oldHz = SourceCoercion.PollHzProvider;
    private readonly Func<int, string, MappingSource, string, MappingSource> _oldCompanion = SourceEvaluator.GyroStickCompanionProvider;
    private readonly MappingSet[] _oldSets = SettingsManager.SlotMappingSets;
    private SourceCoercion.GyroTuning _tuning = new() { SensH = 1, SensV = 1, MinimumOutput01 = .25f };

    public GyroStickDirectionTests()
    {
        SourceCoercion.GyroTuningProvider = (_, _) => _tuning;
        SourceCoercion.GyroBiasProvider = null;
        SourceCoercion.GyroAuxBiasProvider = null;
        SourceCoercion.AimEngageStateProvider = null;
        SourceCoercion.PollHzProvider = () => 1000;
        SourceEvaluator.GyroStickCompanionProvider = InputManager.FindGyroStickCompanion;
        SettingsManager.SlotMappingSets = new MappingSet[InputManager.MaxPads];
    }

    public void Dispose()
    {
        InputManager.ApplyMacroLayerSwitch(0, "Base");
        SourceCoercion.GyroTuningProvider = _oldTuning;
        SourceCoercion.GyroBiasProvider = _oldBias;
        SourceCoercion.GyroAuxBiasProvider = _oldAuxBias;
        SourceCoercion.AimEngageStateProvider = _oldEngage;
        SourceCoercion.PollHzProvider = _oldHz;
        SourceEvaluator.GyroStickCompanionProvider = _oldCompanion;
        SettingsManager.SlotMappingSets = _oldSets;
    }

    private MappingSource Source(string descriptor) => new() { Descriptor = descriptor, DeviceGuid = _device };

    private static CustomInputState State(float pitch, float yaw, float roll)
    {
        var state = new CustomInputState();
        state.Gyro[0] = state.GyroAux[0] = pitch * Deg;
        state.Gyro[1] = state.GyroAux[1] = yaw * Deg;
        state.Gyro[2] = state.GyroAux[2] = roll * Deg;
        return state;
    }

    private static float Read(CustomInputState state, MappingSource source, string target)
        => SourceEvaluator.EvaluateForBipolarAxisTarget(state, source, 0, target, 0, null, .001, source.DeviceGuid);

    [Theory]
    [InlineData(false, "Gyro Horizontal", "Gyro Pitch")]
    [InlineData(true, "Gyro Horizontal", "Gyro Pitch")]
    [InlineData(false, "Gyro L Horizontal", "Gyro L Pitch")]
    [InlineData(true, "Gyro L Horizontal", "Gyro L Pitch")]
    [InlineData(false, "Gyro R Horizontal", "Gyro R Pitch")]
    [InlineData(true, "Gyro R Horizontal", "Gyro R Pitch")]
    public void AddingPitchOrYawRowsDoesNotChangeHorizontalHistory(bool legacy, string horizontal, string pitch)
    {
        _tuning.SensH = _tuning.SensV = 6;
        _tuning.DeadZoneRadPerSec = 3 * Deg;
        _tuning.SmoothingAlpha = legacy ? .9f : 0;
        _tuning.TighteningRadPerSec = legacy ? 0 : 3 * Deg;
        _tuning.SmoothingThresholdRadPerSec = legacy ? 0 : 8 * Deg;
        _tuning.SmoothingWindowSeconds = .05f;
        _tuning.ApplyToPassthrough = true;

        float Run(bool addRows, bool reverse)
        {
            var x = Source(horizontal);
            var y = Source(pitch);
            var yaw = Source(horizontal.Replace("Horizontal", "Yaw"));
            x.DeviceGuid = y.DeviceGuid = yaw.DeviceGuid = Guid.NewGuid().ToString();
            float result = 0;
            var state = State(0, 0, -4);
            for (int frame = 0; frame < 200; frame++)
            {
                SourceCoercion.BeginPollFrame();
                if (addRows && reverse) Read(state, y, "RightThumbAxisY");
                result = Read(state, x, "RightThumbAxisX");
                if (addRows)
                {
                    Read(state, yaw, "LeftThumbAxisX");
                    if (!reverse) Read(state, y, "RightThumbAxisY");
                    SourceCoercion.GetPassthroughGyro(state, x.DeviceGuid, 0, out _, out float auxYaw, out _, aux: true);
                    Assert.Equal(0, auxYaw);
                }
            }
            return result;
        }

        float alone = Run(false, false);
        Assert.InRange(Math.Abs(alone), .258f, .260f);
        Assert.Equal(alone, Run(true, false), 5);
        Assert.Equal(alone, Run(true, true), 5);
    }

    [Fact]
    public void HorizontalChangesContinuouslyAcrossTheOldWinnerBoundary()
    {
        _tuning.MinimumOutput01 = 0;
        var source = Source("Gyro Horizontal");
        float before = Read(State(0, 5, 4.99f), source, "RightThumbAxisX");
        float at = Read(State(0, 5, 5), source, "RightThumbAxisX");
        float after = Read(State(0, 5, 5.01f), source, "RightThumbAxisX");
        Assert.True(before < 0 && after > 0);
        Assert.Equal(0f, at);
        Assert.InRange(Math.Abs(after - before), 0, .001f);
    }

    [Fact]
    public void RadialMinimumPreservesEveryDirectionIncludingNearTheCardinals()
    {
        for (int degrees = 0; degrees < 360; degrees++)
        {
            float x = .2f * MathF.Cos(degrees * Deg), y = .2f * MathF.Sin(degrees * Deg);
            var output = SourceEvaluator.ApplyGyroMinimumOutput(x, y, .25f);
            Assert.Equal(.4f, MathF.Sqrt(output.X * output.X + output.Y * output.Y), 5);
            Assert.InRange(Math.Abs(x * output.Y - y * output.X), 0f, .000001f);
        }
        var nearX = SourceEvaluator.ApplyGyroMinimumOutput(.2f, .001f, .25f);
        Assert.InRange(nearX.Y, .0019f, .0021f);
        Assert.Equal((0f, 0f), SourceEvaluator.ApplyGyroMinimumOutput(0, 0, 1));
        var full = SourceEvaluator.ApplyGyroMinimumOutput(.3f, .4f, 1);
        Assert.Equal(.6f, full.X, 5);
        Assert.Equal(.8f, full.Y, 5);
    }

    private (MappingSource X, MappingSource Y) MapPair(string horizontal = "Gyro Horizontal", string vertical = "Gyro Pitch")
    {
        var x = Source(horizontal);
        var y = Source(vertical);
        SettingsManager.SlotMappingSets[0] = new MappingSet
        {
            Rows =
            {
                new MappingRow { Target = "RightThumbAxisX", Sources = { Source("Axis 3"), x } },
                new MappingRow { Target = "RightThumbAxisY", Sources = { Source("Axis 4"), y } },
            },
        };
        return (x, y);
    }

    [Theory]
    [InlineData("Gyro Horizontal", "Gyro Pitch")]
    [InlineData("Gyro Yaw", "Gyro Pitch")]
    [InlineData("Gyro L Horizontal", "Gyro L Pitch")]
    [InlineData("Gyro R Horizontal", "Gyro R Pitch")]
    public void ActualMappedGyroPairKeepsItsDirectionAndUsesBothSourcesShaping(string horizontal, string vertical)
    {
        var (x, y) = MapPair(horizontal, vertical);
        y.GyroSensitivity = 2;
        y.ParamCurveExponent = .5;
        y.Invert = true;
        var state = State(.5f, 100, 0);
        _tuning.MinimumOutput01 = 0;
        float baseX = Read(state, x, "RightThumbAxisX"), baseY = Read(state, y, "RightThumbAxisY");
        _tuning.MinimumOutput01 = .25f;
        SourceCoercion.BeginPollFrame();
        float outX = Read(state, x, "RightThumbAxisX"), outY = Read(state, y, "RightThumbAxisY");
        Assert.Equal(outX / baseX, outY / baseY, 5);
        Assert.True(Math.Abs(outY) < .25f);
        float radius = MathF.Sqrt(baseX * baseX + baseY * baseY);
        Assert.Equal(.25f + .75f * radius, MathF.Sqrt(outX * outX + outY * outY), 5);
        Assert.Equal(outY, Read(state, y, "RightThumbAxisY"));
        Assert.Equal(outX, Read(state, x, "RightThumbAxisX"));
    }

    [Fact]
    public void CompanionLookupExcludesOtherDevicesSensorsAndInactiveRows()
    {
        var (x, y) = MapPair();
        Assert.Same(y, InputManager.FindGyroStickCompanion(0, "RightThumbAxisX", x, _device));
        y.DeviceGuid = Guid.NewGuid().ToString();
        Assert.Null(InputManager.FindGyroStickCompanion(0, "RightThumbAxisX", x, _device));
        y.DeviceGuid = _device;
        y.Descriptor = "Gyro L Pitch";
        Assert.Null(InputManager.FindGyroStickCompanion(0, "RightThumbAxisX", x, _device));
        y.Descriptor = "Gyro Pitch";
        var set = SettingsManager.SlotMappingSets[0];
        set.ShiftActivators.Add(new ShiftActivator { LayerMask = "Shift", InheritUnmapped = true });
        set.Rows.Add(new MappingRow { Target = "RightThumbAxisY", LayerMask = "Shift", NoInherit = true });
        InputManager.ApplyMacroLayerSwitch(0, "Shift");
        Assert.Null(InputManager.FindGyroStickCompanion(0, "RightThumbAxisX", x, _device));
        InputManager.ApplyMacroLayerSwitch(0, "Base");
        Assert.Same(y, InputManager.FindGyroStickCompanion(0, "RightThumbAxisX", x, _device));
    }

    [Fact]
    public void AmbiguousMultipleGyroSourcesAreNotArbitrarilyPaired()
    {
        var (x, y) = MapPair();
        SettingsManager.SlotMappingSets[0].Rows[0].Sources.Add(Source("Gyro Yaw"));
        Assert.Null(InputManager.FindGyroStickCompanion(0, "RightThumbAxisX", x, _device));
        Assert.Null(InputManager.FindGyroStickCompanion(0, "RightThumbAxisY", y, _device));
    }

    [Fact]
    public void GatedOrUnmappedCompanionDoesNotReduceSingleAxisOutput()
    {
        var (x, y) = MapPair();
        y.GateDescriptor = "Button 0";
        var state = State(100, 100, 0);
        Assert.Equal(-.4f, Read(state, x, "RightThumbAxisX"), 5);
        Assert.Equal(0, Read(state, y, "RightThumbAxisY"));
        SourceCoercion.AimEngageStateProvider = _ => false;
        Assert.Equal(0, Read(state, x, "RightThumbAxisX"));
    }
}
