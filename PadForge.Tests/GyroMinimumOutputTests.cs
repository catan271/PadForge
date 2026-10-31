using System;
using System.Collections.Generic;
using System.IO;
using System.Reflection;
using System.Xml.Serialization;
using PadForge.Common.Input;
using PadForge.Engine;
using PadForge.Engine.Common.Mapping;
using PadForge.Engine.Data;
using PadForge.Services;
using PadForge.ViewModels;
using Xunit;

namespace PadForge.Tests;

[Collection("SettingsManagerStatics")]
public class GyroMinimumOutputTests : IDisposable
{
    private const string Device = "55000000-0000-0000-0000-000000000001";
    private const float DegreesToRadians = MathF.PI / 180f;
    private readonly Func<string, int, SourceCoercion.GyroTuning> _oldTuning = SourceCoercion.GyroTuningProvider;
    private readonly Func<int, bool> _oldEngage = SourceCoercion.AimEngageStateProvider;
    private readonly Func<string, int, (float, float, float)> _oldBias = SourceCoercion.GyroBiasProvider;
    private readonly Func<string, int, (float, float, float)> _oldAuxBias = SourceCoercion.GyroAuxBiasProvider;
    private readonly Func<int, string, MappingSource, string, MappingSource> _oldCompanion = SourceEvaluator.GyroStickCompanionProvider;
    private SourceCoercion.GyroTuning _tuning = new() { SensH = 1, SensV = 1, MinimumOutput01 = 0.2f };

    public GyroMinimumOutputTests()
    {
        SourceCoercion.GyroTuningProvider = (device, slot) => device == Device && slot == 0 ? _tuning
            : new SourceCoercion.GyroTuning { SensH = 1, SensV = 1 };
        SourceCoercion.AimEngageStateProvider = null;
        SourceCoercion.GyroBiasProvider = null;
        SourceCoercion.GyroAuxBiasProvider = null;
        SourceEvaluator.GyroStickCompanionProvider = null;
    }

    public void Dispose()
    {
        SourceCoercion.GyroTuningProvider = _oldTuning;
        SourceCoercion.AimEngageStateProvider = _oldEngage;
        SourceCoercion.GyroBiasProvider = _oldBias;
        SourceCoercion.GyroAuxBiasProvider = _oldAuxBias;
        SourceEvaluator.GyroStickCompanionProvider = _oldCompanion;
    }

    private static CustomInputState State(float rate)
    {
        var state = new CustomInputState();
        state.Gyro[0] = state.Gyro[1] = state.Gyro[2] = rate * DegreesToRadians;
        state.GyroAux[0] = state.GyroAux[1] = state.GyroAux[2] = rate * DegreesToRadians;
        return state;
    }

    private static MappingSource Source(string descriptor = "Gyro Yaw") => new() { DeviceGuid = Device, Descriptor = descriptor };

    private static float Stick(CustomInputState state, MappingSource source, string target = "RightThumbAxisX", int slot = 0)
        => SourceEvaluator.EvaluateForBipolarAxisTarget(state, source, slot, target, 0, null, 0.001, Device);

    [Theory]
    [InlineData(0f, 0f)]
    [InlineData(0.01f, 0.200016f)]
    [InlineData(250f, 0.6f)]
    [InlineData(-250f, -0.6f)]
    [InlineData(500f, 1f)]
    [InlineData(1000f, 1f)]
    public void ActiveOutputRescalesWhileZeroAndDirectionArePreserved(float rate, float expected)
        // Yaw uses the existing right-hand-rule to stick-frame sign correction.
        => Assert.Equal(-expected, Stick(State(rate), Source()), 5);

    [Theory]
    [InlineData("LeftThumbAxisX")]
    [InlineData("LeftThumbAxisY")]
    [InlineData("RightThumbAxisX")]
    [InlineData("RightThumbAxisY")]
    public void AllFourStickAxesOptIn(string target)
        => Assert.Equal(0.6f, Math.Abs(Stick(State(250), Source(), target)), 5);

    [Theory]
    [InlineData("Gyro Pitch")]
    [InlineData("Gyro Roll")]
    [InlineData("Gyro Horizontal")]
    [InlineData("Gyro L Yaw")]
    [InlineData("Gyro R Yaw")]
    public void RateGyroFamiliesShareTheMinimum(string descriptor)
    {
        var state = State(250);
        if (descriptor == "Gyro Horizontal") state.Gyro[2] = 0;
        Assert.Equal(0.6f, Math.Abs(Stick(state, Source(descriptor))), 5);
    }

    [Theory]
    [InlineData("KbmMouseX")]
    [InlineData("KbmMouseY")]
    [InlineData("KbmScroll")]
    [InlineData("KbmScrollH")]
    [InlineData("RawAxis0")]
    [InlineData("MotionYaw")]
    [InlineData("TouchpadX")]
    public void OtherAxisTargetsDoNotOptIn(string target)
        => Assert.Equal(0.5f, Math.Abs(Stick(State(250), Source(), target)), 5);

    [Fact]
    public void DeadzoneAndEngagementStillReturnZero()
    {
        _tuning.DeadZoneRadPerSec = 3 * DegreesToRadians;
        Assert.Equal(0, Stick(State(2), Source()));
        Assert.Equal(0, Stick(State(3), Source()));
        Assert.InRange(Math.Abs(Stick(State(3.1f), Source())), 0.2f, 0.201f);
        SourceCoercion.AimEngageStateProvider = _ => false;
        Assert.Equal(0, Stick(State(250), Source()));
    }

    [Fact]
    public void MinimumFollowsResponseAndSourceShapingAndKeepsInversion()
    {
        var source = Source();
        source.ParamCurveExponent = 2;
        Assert.Equal(-0.4f, Stick(State(250), source), 5);
        source.Invert = true;
        Assert.Equal(0.4f, Stick(State(250), source), 5);
        _tuning.InvertYaw = true;
        Assert.Equal(-0.4f, Stick(State(250), source), 5);
    }

    [Fact]
    public void NonGyroStickSourcesRemainUnchanged()
    {
        var state = State(0);
        state.Axis[0] = 32768 + 16384;
        Assert.Equal(0.5f, Stick(state, Source("Axis 0")), 3);
    }

    [Fact]
    public void TiltAndLeanRemainUnchangedWhileDeflected()
    {
        var oldGravity = SourceCoercion.GravityProvider;
        try
        {
            SourceCoercion.ResetGyroLeanNeutral();
            var gravity = (0f, 9.81f, 0f);
            SourceCoercion.GravityProvider = _ => gravity;
            var tilt = Source(SourceCoercion.GyroTiltXDescriptor);
            var lean = Source("Gyro Lean X");
            Assert.Equal(0f, Stick(State(0), tilt));
            Assert.Equal(0f, Stick(State(0), lean));
            float angle = 12.5f * DegreesToRadians;
            gravity = (-9.81f * MathF.Sin(angle), 9.81f * MathF.Cos(angle), 0f);
            float tilted = Stick(State(0), tilt);
            float leaned = Stick(State(0), lean);
            Assert.NotEqual(0f, tilted);
            Assert.NotEqual(0f, leaned);
            _tuning.MinimumOutput01 = 0;
            Assert.Equal(tilted, Stick(State(0), tilt));
            Assert.Equal(leaned, Stick(State(0), lean));
        }
        finally
        {
            SourceCoercion.GravityProvider = oldGravity;
            SourceCoercion.ResetGyroLeanNeutral();
        }
    }

    [Fact]
    public void FullMinimumOnlySaturatesActiveOutput()
    {
        _tuning.MinimumOutput01 = 1;
        Assert.Equal(-1f, Stick(State(1), Source()));
        Assert.Equal(1f, Stick(State(-1), Source()));
        Assert.Equal(0f, Stick(State(0), Source()));
    }

    [Theory]
    [InlineData(-1f, 0.5f)]
    [InlineData(float.NaN, 0.5f)]
    [InlineData(float.PositiveInfinity, 0.5f)]
    [InlineData(2f, 1f)]
    public void EngineAlsoBoundsTheMinimum(float minimum, float expected)
    {
        _tuning.MinimumOutput01 = minimum;
        Assert.Equal(expected, Math.Abs(Stick(State(250), Source())), 5);
    }

    [Fact]
    public void ZeroMinimumIsIdenticalToExistingCoercionAndIsPerDeviceAndSlot()
    {
        Assert.Equal(-0.5f, Stick(State(250), Source(), slot: 1), 5);
        var otherDevice = Source();
        otherDevice.DeviceGuid = "55000000-0000-0000-0000-000000000002";
        Assert.Equal(-0.5f, Stick(State(250), otherDevice), 5);
        _tuning.MinimumOutput01 = 0;
        Assert.Equal(-SourceCoercion.EvaluateForBipolarAxisTarget(State(250), Source(), 0), Stick(State(250), Source()));
    }

    [Fact]
    public void MouseTriggerButtonTiltAndPassthroughRemainUnchanged()
    {
        var state = State(250);
        float trigger = SourceCoercion.EvaluateForTriggerTarget(state, Source(), 0);
        bool button = SourceCoercion.EvaluateForButtonTarget(state, Source(), 25, 0);
        var mouse = SourceCoercion.ReadGyroMouseCounts(state, Source(), 0, Device, 0.001f, true);
        float tilt = Stick(state, Source("Gyro Lean X"));
        _tuning.ApplyToPassthrough = true;
        SourceCoercion.GetPassthroughGyro(state, Device, 0, out float p, out float y, out float r);
        _tuning.MinimumOutput01 = 0;
        Assert.Equal(trigger, SourceCoercion.EvaluateForTriggerTarget(state, Source(), 0));
        Assert.Equal(button, SourceCoercion.EvaluateForButtonTarget(state, Source(), 25, 0));
        Assert.Equal(mouse, SourceCoercion.ReadGyroMouseCounts(state, Source(), 0, Device, 0.001f, true));
        Assert.Equal(tilt, Stick(state, Source("Gyro Lean X")));
        SourceCoercion.GetPassthroughGyro(state, Device, 0, out float p0, out float y0, out float r0);
        Assert.Equal((p0, y0, r0), (p, y, r));
    }

    [Theory]
    [InlineData("20", 0.2f)]
    [InlineData("-1", 0f)]
    [InlineData("200", 1f)]
    [InlineData("NaN", 0f)]
    [InlineData("Infinity", 0f)]
    [InlineData("invalid", 0f)]
    [InlineData(null, 0f)]
    public void StoredPercentIsSanitizedAndNormalized(string stored, float expected)
        => Assert.Equal(expected, InputService.GyroTuningFromPadSetting(new PadSetting { GyroMinimumOutput = stored }).MinimumOutput01);

    [Fact]
    public void NondefaultMinimumAppearsInTheGyroSummary()
    {
        var append = typeof(InputService).GetMethod("AppendGyroStageTokens", BindingFlags.NonPublic | BindingFlags.Static);
        var tokens = new List<string>();
        append.Invoke(null, new object[] { tokens, new PadSetting(), false });
        Assert.DoesNotContain(tokens, token => token.StartsWith("MIN "));
        append.Invoke(null, new object[] { tokens, new PadSetting { GyroMinimumOutput = "20.5" }, false });
        Assert.Contains("MIN 20.5%", tokens);
    }

    [Fact]
    public void BothSettingsSyncPairsCarryTheMinimum()
    {
        var devices = SettingsManager.UserDevices;
        var settings = SettingsManager.UserSettings;
        try
        {
            SettingsManager.UserDevices = new DeviceCollection();
            SettingsManager.UserSettings = new SettingsCollection();
            var device = new UserDevice { InstanceGuid = Guid.Parse(Device), IsOnline = true, HasGyro = true };
            SettingsManager.UserDevices.Items.Add(device);
            var assignment = new UserSetting { InstanceGuid = device.InstanceGuid, MapTo = 0 };
            var setting = new PadSetting { GyroMinimumOutput = "20.5" };
            assignment.SetPadSetting(setting);
            SettingsManager.UserSettings.Items.Add(assignment);
            var main = new MainViewModel();
            var input = new InputService(main);
            var store = new SettingsService(main);
            var vm = main.Pads[0];
            var mapped = new PadViewModel.MappedDeviceInfo { InstanceGuid = device.InstanceGuid, Name = "Gyro test", IsOnline = true };
            vm.MappedDevices.Add(mapped);
            vm.SelectedMappedDevice = mapped;
            InputService.LoadPadSettingIntoViewModel(vm, setting);
            Assert.Equal(20.5, vm.GyroMinimumOutput);
            assignment.SetPadSetting(new PadSetting());
            typeof(InputService).GetMethod("SaveViewModelToPadSetting", BindingFlags.NonPublic | BindingFlags.Instance)
                .Invoke(input, new object[] { vm, device.InstanceGuid, false });
            Assert.Equal("20.5", assignment.GetPadSetting().GyroMinimumOutput);
            vm.GyroMinimumOutput = 0;
            assignment.SetPadSetting(setting);
            typeof(SettingsService).GetMethod("LoadPadSettings", BindingFlags.NonPublic | BindingFlags.Instance)
                .Invoke(store, new object[] { new[] { assignment }, new[] { setting } });
            Assert.Equal(20.5, vm.GyroMinimumOutput);
            assignment.SetPadSetting(new PadSetting());
            store.UpdatePadSettingsFromViewModels();
            Assert.Equal("20.5", assignment.GetPadSetting().GyroMinimumOutput);
        }
        finally
        {
            SettingsManager.UserDevices = devices;
            SettingsManager.UserSettings = settings;
        }
    }

    [Fact]
    public void SettingsSurviveXmlCloneAndCopyAndAffectChecksum()
    {
        var serializer = new XmlSerializer(typeof(PadSetting));
        using var legacy = new StringReader("<PadSetting />");
        Assert.Equal("0", ((PadSetting)serializer.Deserialize(legacy)).GyroMinimumOutput);
        var setting = new PadSetting { GyroMinimumOutput = "20" };
        using var writer = new StringWriter();
        serializer.Serialize(writer, setting);
        using var reader = new StringReader(writer.ToString());
        Assert.Equal("20", ((PadSetting)serializer.Deserialize(reader)).GyroMinimumOutput);
        Assert.Equal("20", setting.CloneDeep().GyroMinimumOutput);
        var copy = new PadSetting();
        copy.CopyFrom(setting);
        Assert.Equal("20", copy.GyroMinimumOutput);
        Assert.NotEqual(new PadSetting().ComputeChecksum(), setting.ComputeChecksum());
    }

    [Theory]
    [InlineData(-1, 0)]
    [InlineData(20.5, 20.5)]
    [InlineData(101, 100)]
    [InlineData(double.NaN, 0)]
    [InlineData(double.PositiveInfinity, 0)]
    public void ViewModelBoundsAndBothResets(double input, double expected)
    {
        var vm = new PadViewModel(0) { GyroMinimumOutput = input };
        Assert.Equal(expected, vm.GyroMinimumOutput);
        vm.GyroMinimumOutput = 20;
        vm.ResetGyroMinimumOutputCommand.Execute(null);
        Assert.Equal(0, vm.GyroMinimumOutput);
        vm.GyroMinimumOutput = 20;
        vm.ResetGyroResponseCardCommand.Execute(null);
        Assert.Equal(0, vm.GyroMinimumOutput);
    }
}
