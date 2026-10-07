"""Resolve tactile layouts from registered physical model numbers, not table length."""
from pathlib import Path

import yaml


def load_profiles(path):
    data = yaml.safe_load(Path(path).read_text())
    if not isinstance(data, dict) or set(data) != {'controllers', 'sensors'}:
        raise ValueError('Model profiles must contain controllers and sensors mappings')
    profiles = {}
    for kind in ('controllers', 'sensors'):
        entries = data[kind]
        if not isinstance(entries, dict):
            raise ValueError(f'{kind} must be a mapping')
        profiles[kind] = {}
        for number, entry in entries.items():
            if type(number) is not int or not 0 <= number <= 65535:
                raise ValueError(f'Invalid physical model number: {number!r}')
            keys = {'num_taxels', 'side'} if kind == 'controllers' else {'num_taxels'}
            if not isinstance(entry, dict) or set(entry) != keys:
                raise ValueError(f'Invalid {kind} profile for model {number}')
            if type(entry['num_taxels']) is not int or entry['num_taxels'] not in (9, 10):
                raise ValueError(f'Model {number} must specify 9 or 10 taxels')
            if kind == 'controllers' and entry['side'] not in ('left', 'right'):
                raise ValueError(f'Model {number} must specify left or right')
            profiles[kind][number] = entry
    return profiles


def resolve_num_taxels(profiles, side, controller_model, sensor_models=None):
    if side not in ('left', 'right'):
        raise ValueError(f'Invalid hand side: {side}')
    controller = profiles['controllers'].get(controller_model)
    if controller is not None:
        if controller['side'] != side:
            raise ValueError(f'Model {controller_model} is for the {controller["side"]} hand')
        if sensor_models is None:
            return controller['num_taxels']
    if sensor_models is None or len(sensor_models) != 5:
        raise ValueError(f'Unregistered controller model {controller_model}; need five sensor models')
    counts = []
    for number in sensor_models:
        sensor = profiles['sensors'].get(number)
        if sensor is None:
            raise ValueError(f'Unregistered tactile model {number}; register the official model number')
        counts.append(sensor['num_taxels'])
    if len(set(counts)) != 1:
        raise ValueError('Mixed 9/10-channel tactile sensors are not supported by this hand profile')
    if controller is not None and controller['num_taxels'] != counts[0]:
        raise ValueError('Controller and tactile model profiles disagree')
    return counts[0]


def probe_num_taxels(port_name, baud_rate, side, profiles, sdk=None):
    """Ping only; release the serial port before DHI opens it. Never write registers."""
    if side not in ('left', 'right'):
        raise ValueError(f'Invalid hand side: {side}')
    if sdk is None:
        try:
            import dynamixel_sdk as sdk
        except ImportError as error:
            raise RuntimeError('Auto detection requires the DynamixelSDK Python package') from error
    port = sdk.PortHandler(port_name)
    packet = sdk.PacketHandler(2.0)
    base_id = 140 if side == 'left' else 110

    def ping(device_id):
        number, result, error = packet.ping(port, device_id)
        if result != sdk.COMM_SUCCESS or error:
            raise RuntimeError(
                f'Model probe failed at ID {device_id}: communication={result}, device_error={error}')
        return number

    opened = False
    try:
        if not port.openPort():
            raise RuntimeError(f'Cannot open model probe port {port_name}')
        opened = True
        if not port.setBaudRate(baud_rate):
            raise RuntimeError(f'Cannot set model probe baud rate {baud_rate}')
        controller_model = ping(base_id)
        if controller_model in profiles['controllers']:
            return resolve_num_taxels(profiles, side, controller_model)
        sensor_models = [ping(base_id + offset) for offset in (5, 10, 15, 20, 25)]
        return resolve_num_taxels(profiles, side, controller_model, sensor_models)
    except OSError as error:
        raise RuntimeError(f'Model probe serial error on {port_name}: {error}') from error
    finally:
        if opened:
            port.closePort()


def configure_tactile_layout(context, *, side):
    """Launch preflight: resolve before xacro, broadcaster or any hardware node starts."""
    from launch.actions import SetLaunchConfiguration
    from launch.logging import get_logger
    from launch.substitutions import LaunchConfiguration

    value = LaunchConfiguration('num_taxels').perform(context)
    if value != 'auto':
        return []
    for name in ('use_mock_hardware', 'use_sim'):
        if LaunchConfiguration(name).perform(context).lower() in ('true', '1'):
            raise ValueError('Use explicit num_taxels:=9 or 10 for mock hardware and simulation')
    profiles = load_profiles(LaunchConfiguration('tactile_model_profiles').perform(context))
    if not any(profiles.values()):
        raise ValueError('No official model numbers registered in tactile_model_profiles')
    count = probe_num_taxels(
        LaunchConfiguration('port_name').perform(context), 4000000, side, profiles)
    get_logger('tactile_model').info(f'Physical model profile selected {count} channels ({side})')
    return [SetLaunchConfiguration('num_taxels', str(count))]
