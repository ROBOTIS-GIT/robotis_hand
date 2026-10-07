"""Check that hardware, mock sensors and broadcaster configurations agree."""
from pathlib import Path
import subprocess
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
import pytest
import yaml


@pytest.mark.parametrize('revision', ['rev1', 'rev2'])
@pytest.mark.parametrize('side', ['left', 'right'])
@pytest.mark.parametrize('count', [9, 10])
def test_tactile_configuration(revision, side, count):
    root = Path(__file__).resolve().parents[2]
    description = (root / 'robotis_hand_description' / 'urdf'
                   / f'hx5_d20_{revision}' / f'hx5_d20_{side}.xacro')
    xml = subprocess.check_output([
        'xacro', str(description), f'num_taxels:={count}', 'use_mock_hardware:=true',
    ], text=True)
    tree = ET.fromstring(xml)
    sensors = tree.findall('.//ros2_control/sensor')
    gpios = [p for p in tree.findall('.//ros2_control/gpio') if 'sensor' in p.attrib['name']]
    assert len(sensors) == 5
    assert len(gpios) == 5
    names = [f'Present Pressure {i}' for i in range(1, count + 1)]
    for sensor in sensors + gpios:
        assert [i.attrib['name'] for i in sensor.findall('state_interface')] == names
    config = yaml.safe_load((root / 'robotis_hand_bringup' / 'config' / 'hx5_d20'
                             / f'pressure_{count}.yaml').read_text())
    params = config[f'/**/pressure_{side[0]}_broadcaster']['ros__parameters']
    assert params['interface_names'] == names
    assert len(params['sensor_names']) == 5


@pytest.mark.parametrize('revision', ['rev1', 'rev2'])
@pytest.mark.parametrize('side', ['left', 'right'])
def test_both_channel_counts_use_the_same_dhi_tables(revision, side):
    root = Path(__file__).resolve().parents[2]
    description = (root / 'robotis_hand_description' / 'urdf'
                   / f'hx5_d20_{revision}' / f'hx5_d20_{side}.xacro')
    driver_share = Path(get_package_share_directory('dynamixel_hardware_interface'))
    profiles = []
    for count in (9, 10):
        xml = subprocess.check_output([
            'xacro', str(description), f'num_taxels:={count}',
        ], text=True)
        tree = ET.fromstring(xml)
        for index in range(1, 6):
            size = tree.find(f'.//gpio/param[@name="SyncTable {index} Read Size 5"]')
            assert int(size.text) == count
        folder = tree.find('.//hardware/param[@name="dynamixel_model_folder"]').text
        assert folder == '/param/dxl_model'
        table_directory = driver_share / folder.lstrip('/')
        model_files = dict(
            line.split() for line in (table_directory / 'dynamixel.model').read_text()
            .splitlines()[1:] if line.strip()
        )
        sensor_profiles = []
        for gpio in tree.findall('.//ros2_control/gpio'):
            parameters = {item.attrib['name']: item.text for item in gpio.findall('param')}
            if parameters.get('type') != 'virtual_sensor':
                continue
            model_file = table_directory / model_files[parameters['model_num']]
            control_table = model_file.read_text().split('[control table]', 1)[1]
            pressure_items = {}
            for line in control_table.splitlines():
                columns = line.split(maxsplit=2)
                if len(columns) == 3 and columns[2].startswith('Present Pressure '):
                    pressure_items[columns[2]] = (int(columns[0]), int(columns[1]))
            assert set(pressure_items) == {
                f'Present Pressure {channel}' for channel in range(1, 11)
            }
            requested = [item.attrib['name'] for item in gpio.findall('state_interface')]
            assert len(requested) == count
            assert set(requested) <= pressure_items.keys()
            assert all(pressure_items[name][1] == 1 for name in requested)
            if count == 9:
                assert 'Present Pressure 10' not in requested
            sensor_profiles.append((
                gpio.attrib['name'], parameters['comm_id'], parameters['ID'],
                parameters['model_num'], str(model_file), pressure_items,
            ))
        assert len(sensor_profiles) == 5
        profiles.append(sensor_profiles)
    assert profiles[0] == profiles[1]


@pytest.mark.parametrize('side', ['rr', 'rl'])
@pytest.mark.parametrize('count', [9, 10])
def test_hx5_indirect_read_and_write_regions_do_not_overlap(side, count):
    driver_share = Path(get_package_share_directory('dynamixel_hardware_interface'))
    model_file = driver_share / 'param' / 'dxl_model' / f'hx5_d20_{side}.model'
    entries = {}
    for line in model_file.read_text().splitlines():
        columns = line.split(maxsplit=2)
        if len(columns) == 3 and columns[0].isdigit():
            entries[columns[2]] = int(columns[0])
    read_bytes = 20 * 6 + 5 * count
    write_bytes = 20 * 4
    mapping_start = entries['Indirect Address Read']
    data_start = entries['Indirect Data Read']
    write_slot = (entries['Indirect Address Write'] - mapping_start) // 2
    assert (entries['Indirect Address Write'] - mapping_start) % 2 == 0
    assert entries['Indirect Data Write'] - data_start == write_slot
    assert read_bytes <= write_slot
    assert write_slot + write_bytes <= 256
