"""Synthetic model IDs below are fixtures, not allocated hardware identities."""
import pytest

from launch import LaunchContext
from robotis_hand_bringup.tactile_model import (
    configure_tactile_layout, load_profiles, probe_num_taxels, resolve_num_taxels,
)


@pytest.fixture
def profiles():
    return {
        'controllers': {
            50000: {'side': 'right', 'num_taxels': 9},
            50001: {'side': 'right', 'num_taxels': 10},
            50002: {'side': 'left', 'num_taxels': 10},
        },
        'sensors': {50003: {'num_taxels': 9}, 50004: {'num_taxels': 10}},
    }


@pytest.mark.parametrize('model,count', [(50000, 9), (50001, 10)])
def test_controller_identity(profiles, model, count):
    assert resolve_num_taxels(profiles, 'right', model) == count


@pytest.mark.parametrize('model,count', [(50003, 9), (50004, 10)])
def test_sensor_identity_without_controller_split(profiles, model, count):
    assert resolve_num_taxels(profiles, 'right', 260, [model] * 5) == count


@pytest.mark.parametrize('controller,sensors,message', [
    (260, None, 'Unregistered controller'),
    (260, [253] * 5, 'Unregistered tactile'),
    (260, [50003] * 4, 'five sensor'),
    (260, [50003] * 4 + [50004], 'Mixed'),
    (50001, [50003] * 5, 'disagree'),
    (50002, None, 'left hand'),
])
def test_ambiguous_models_are_rejected(profiles, controller, sensors, message):
    with pytest.raises(ValueError, match=message):
        resolve_num_taxels(profiles, 'right', controller, sensors)


class FakeSDK:
    COMM_SUCCESS = 0

    def __init__(self, models, failed_id=None):
        self.models = models
        self.failed_id = failed_id
        self.closed = False
        self.pinged = []

    def PortHandler(self, path):
        assert path == '/test-port'
        return self

    def PacketHandler(self, protocol):
        assert protocol == 2.0
        return self

    def openPort(self):
        return True

    def setBaudRate(self, baud):
        assert baud == 4000000
        return True

    def ping(self, port, device_id):
        self.pinged.append(device_id)
        return self.models[device_id], (-1 if device_id == self.failed_id else 0), 0

    def closePort(self):
        self.closed = True


@pytest.mark.parametrize('side,base', [('right', 110), ('left', 140)])
def test_sensor_probe_ids_and_port_release(profiles, side, base):
    sdk = FakeSDK({base: 260, **{base + i: 50004 for i in (5, 10, 15, 20, 25)}})
    assert probe_num_taxels('/test-port', 4000000, side, profiles, sdk) == 10
    assert sdk.pinged == [base, base + 5, base + 10, base + 15, base + 20, base + 25]
    assert sdk.closed


def test_controller_probe_releases_port(profiles):
    sdk = FakeSDK({110: 50000})
    assert probe_num_taxels('/test-port', 4000000, 'right', profiles, sdk) == 9
    assert sdk.pinged == [110]
    assert sdk.closed


def test_failed_ping_releases_port(profiles):
    sdk = FakeSDK({110: 50001}, failed_id=110)
    with pytest.raises(RuntimeError, match='probe failed'):
        probe_num_taxels('/test-port', 4000000, 'right', profiles, sdk)
    assert sdk.closed


def test_missing_port_keeps_the_original_error(profiles, monkeypatch):
    sdk = FakeSDK({110: 50001})

    def missing_port():
        raise FileNotFoundError('device disconnected')

    monkeypatch.setattr(sdk, 'openPort', missing_port)
    with pytest.raises(RuntimeError, match='device disconnected'):
        probe_num_taxels('/test-port', 4000000, 'right', profiles, sdk)
    assert not sdk.closed


def test_baud_failure_releases_open_port(profiles, monkeypatch):
    sdk = FakeSDK({110: 50001})
    monkeypatch.setattr(sdk, 'setBaudRate', lambda baud: False)
    with pytest.raises(RuntimeError, match='baud rate'):
        probe_num_taxels('/test-port', 4000000, 'right', profiles, sdk)
    assert sdk.closed


@pytest.mark.parametrize('contents', [
    'controllers: {}\nsensors: {50004: {num_taxels: 8}}',
    'controllers: {50001: {side: unknown, num_taxels: 10}}\nsensors: {}',
    'controllers: {}\nsensors: {70000: {num_taxels: 10}}',
    'controllers: {}\nsensors: {50004: {num_taxels: true}}',
])
def test_invalid_registry(tmp_path, contents):
    path = tmp_path / 'profiles.yaml'
    path.write_text(contents)
    with pytest.raises(ValueError):
        load_profiles(path)


def test_preflight_sets_channel_count_before_nodes(tmp_path, monkeypatch):
    path = tmp_path / 'profiles.yaml'
    path.write_text('controllers: {50001: {side: right, num_taxels: 10}}\nsensors: {}')
    context = LaunchContext()
    context.launch_configurations.update(
        num_taxels='auto', use_mock_hardware='false', use_sim='false',
        port_name='/test-port', tactile_model_profiles=str(path))
    monkeypatch.setattr('robotis_hand_bringup.tactile_model.probe_num_taxels',
                        lambda *args: 10)
    actions = configure_tactile_layout(context, side='right')
    actions[0].execute(context)
    assert context.launch_configurations['num_taxels'] == '10'


@pytest.mark.parametrize('count', ['9', '10'])
def test_explicit_selection_never_probes(count):
    context = LaunchContext()
    context.launch_configurations['num_taxels'] = count
    assert configure_tactile_layout(context, side='right') == []


def test_current_hand_identity_is_not_mistaken_for_a_layout(profiles):
    sdk = FakeSDK({110: 260, **{i: 253 for i in (115, 120, 125, 130, 135)}})
    with pytest.raises(ValueError, match='Unregistered tactile model 253'):
        probe_num_taxels('/test-port', 4000000, 'right', profiles, sdk)
    assert sdk.pinged == [110, 115, 120, 125, 130, 135]
    assert sdk.closed


def test_auto_mock_rejected_before_port_access():
    context = LaunchContext()
    context.launch_configurations.update(num_taxels='auto', use_mock_hardware='true')
    with pytest.raises(ValueError, match='explicit num_taxels'):
        configure_tactile_layout(context, side='right')
