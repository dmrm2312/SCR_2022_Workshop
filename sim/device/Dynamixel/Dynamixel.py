from sim.device.DeviceBase import DeviceBase
from sim.typing import DefDict
from sim.typing import definitions as DEF
from sim.device.Dynamixel.DynamixelTable import DynamixelX
from sim.typing import MapRule as rule
from sim.typing.std.StdUnit import Count, Pos, Vel, Acc, Ang, AngVel, AngAcc

import logging, time
import numpy as np
import dynamixel_sdk as x

DEFAULT_SPEED = 2
DEFAULT_ACC = 20
ID = 'ID'

default_func = (None, None)

motor = DefDict(dict(pos=Ang,
             vel=AngVel(unit='rad/s', drange=('-52rpm', '52rpm'), default=DEFAULT_SPEED),
             acc=Acc(default=DEFAULT_ACC),
             on=bool))

sensor = DefDict(dict(pos=Ang, vel=AngVel(unit='rad/s', drange=('-52rpm', '52rpm'))))


class Dynamixel(DeviceBase):
    device_name = 'Dynamixel'
    def __init__(self, id_lists, slave_ids=None, device_port='/dev/ttyUSB0', offset_func=default_func, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.device_port = device_port
        # motor sensing is only for masters
        # motor input is for all
        self.ids = id_lists
        self.slave_ids = slave_ids
        self.offset_func = offset_func
        self.id_lists = id_lists

        # flags i dont like
        self.read_vel = False
        self.vel_mode = False
        self.to_thread = True

    def init(self, *args, **kwargs):
        if self.slave_ids and self.slave_ids is not None:
            self.ids.extend(self.slave_ids)

            slave_bind = rule(DEF.define(prefix=ID, num=self.id_lists),
                                   None,
                                   DEF.define(prefix=ID, num=self.slave_ids), to_list=True)
            self.drive_space.set_rule(slave_bind)

        self.toDynamixel = rule(None, self.offset_func[0], to_list=True)
        self.fromDynamixel = rule(self.sense_space.pos().list_keys(), self.offset_func[1], to_list=True)

        self.packet = x.PacketHandler(DynamixelX.PROTOCOL_VERSION)
        self.port = x.PortHandler(self.device_port)

    def open(self, *args, **kwargs):
        if not self.port.is_open:
            self.port.openPort()
            self.port.setBaudRate(DynamixelX.BAUDRATE)
        if self.port.is_open:
            logging.info('Dynamixel Connected')
            opened = True
        else:
            logging.error('Connection failed to {}'.format(self.device_port))
            opened = False

        if opened:
            self.restart()
            self.enable(enable=True)
            self.velocity_mode()
            for key, acc in self.drive_space.acc().items():
                if acc == 0:
                    self.drive_space.acc()[key] = DEFAULT_ACC
            self._sync_write(self.drive_space.ID().acc(), DynamixelX.PROFILE_ACCELERATION)

    def close(self, *args, **kwargs):
        if self.port.is_open:
            self.enable(False)
            self.port.closePort()
            logging.info('Dynamixel port closed')

    def homing(self):
        self._sync_write(self.drive_space.ID().pos(), DynamixelX.GOAL_POSITION, np.pi)
        time.sleep(1)   # TODO: wait till the motor reach the target position

    def enable(self, enable, *args, **kwargs):
        if self.port.is_open:
            for i, id in enumerate(self.ids):
                result = self.func_retry(self.packet.write1ByteTxRx,
                                         args=(self.port, id, DynamixelX.TORQUE_ENABLE.ADDR, enable),
                                         success_condition=x.COMM_SUCCESS)
                if result == x.COMM_SUCCESS:
                    self.drive_space.ID(id).on().set(enable)
            if sum(self.drive_space.on().list()) == len(self.drive_space):
                logging.info(f'enabled {self.drive_space.on()}')
                return True
            else:
                if enable:
                    logging.info(f'Could not enabled {self.drive_space.on()}')
                    raise ConnectionError(f"Dynamixel could not enabled {self.drive_space.on()}")
                else:
                    logging.info('Disabled')
        return False

    def drive(self, inpt: DefDict, timestamp, *args, **kwargs):
        # TODO: change this so that each motor could have different mode
        self.drive_space.set(inpt)
        if self.velocity_mode():
            self._sync_write(self.drive_space.ID().vel(), DynamixelX.GOAL_VELOCITY)
        else:
            self._sync_write(self.drive_space.pos().bind(self.toDynamixel).ID(), DynamixelX.GOAL_POSITION)
            self._sync_write(self.drive_space.ID().vel(), DynamixelX.PROFILE_VELOCITY)

    def sense(self, *args, **kwargs):
        if not self.read_vel and not self.vel_mode:
            self._sync_read(self.sense_space.ID().pos(), DynamixelX.PRESENT_POSITION)
            self.sense_space.pos().bind(self.fromDynamixel)
            self.read_vel = True
        else:
            self._sync_read(self.sense_space.ID().vel(), DynamixelX.PRESENT_VELOCITY)
            self.read_vel = False
        return self.sense_space

    def _sync_write(self, values:DefDict, table, value_overwrite=None):
        # TODO use bulk instead (table def added when add params)
        write = x.GroupSyncWrite(self.port, self.packet, table.ADDR, table.LEN)
        for id, val in zip(values.list_keys(), values.list()):
            if value_overwrite is not None: val = value_overwrite
            write.addParam(int(id), table.from_unit(val))
        result = self.func_retry(write.txPacket, success_condition=x.COMM_SUCCESS)
        write.clearParam()

    def _sync_read(self, values: DefDict, table):
        # TODO use bulk instead (table def added when add params)
        read = x.GroupSyncRead(self.port, self.packet, table.ADDR, table.LEN)
        for id, val in zip(values.list_keys(), values.list()):
            read.addParam(int(id))
        read.txRxPacket()
        #self.func_retry(read.txRxPacket, success_condition=x.COMM_SUCCESS)
        for id, key in zip(values.list_keys(), values.list_keys()):
            result = read.isAvailable(int(id), table.ADDR, table.LEN)
            if result:
                # Get Dynamixel present position value
                values[key] = table.to_unit(read.getData(int(id), table.ADDR, table.LEN))
        read.clearParam()
        return values

    def restart(self):
        # Try reboot
        # Dynamixel LED will flicker while it reboots
        for i in self.ids:
            self.func_retry(self.packet.reboot, args=(self.port, i), success_condition=x.COMM_SUCCESS)

    def velocity_mode(self):
        if self.vel_mode is False and any(self.drive_space.pos() == float('inf')):
            self._sync_write(self.drive_space.ID(), DynamixelX.OPERATING_MODE, DynamixelX.OPERATING_MODE.VEL_MODE)
            self.vel_mode = True
        elif self.vel_mode is False:
            self._sync_write(self.drive_space.ID(), DynamixelX.OPERATING_MODE, DynamixelX.OPERATING_MODE.POS_MODE)
            self._sync_write(self.drive_space.ID().vel(), DynamixelX.PROFILE_VELOCITY)
            self.vel_mode = False
        return self.vel_mode

    def system_voltage_status(self):
        voltage = min = self._sync_read(self.drive_space.ID(), DynamixelX.PRESENT_INPUT_VOLTAGE)


    @staticmethod
    def func_retry(func, args=None, retry_max=DynamixelX.RETRY_MAX, success_condition=True):
        result = None
        for retry_count in range(retry_max):
            if args is None:    result = func()
            else:               result = func(*args)
            try:
                iterator = iter(result)
                result = next(iterator)
            except TypeError:
                pass
            if result == success_condition:
                logging.debug(f"Connection try {retry_count} succeed")
                break
        return result

    @staticmethod
    def create_drive_space(IDs, *args, **kwargs):
        d = DEF.define(prefix=ID, num=IDs, dtype=motor)
        return DefDict(d, name='dynamixel')

    @staticmethod
    def create_sense_space(IDs, *args, **kwargs):
        d = DEF.define(prefix=ID, num=IDs, dtype=sensor)
        return DefDict(d, name='dynamixel')

