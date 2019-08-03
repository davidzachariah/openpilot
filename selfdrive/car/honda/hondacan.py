import struct
import common.numpy_fast as np #Clarity
from selfdrive.config import Conversions as CV
from selfdrive.car.honda.values import CAR, HONDA_BOSCH, VEHICLE_STATE_MSG #Clarity

# *** Honda specific ***
def can_cksum(mm):
  s = 0
  for c in mm:
    c = ord(c)
    s += (c>>4)
    s += c & 0xF
  s = 8-s
  s %= 0x10
  return s


def fix(msg, addr):
  msg2 = msg[0:-1] + chr(ord(msg[-1]) | can_cksum(struct.pack("I", addr)+msg))
  return msg2

#Clarity
def make_can_msg(addr, dat, idx, alt):
  if idx is not None:
    dat += chr(idx << 4)
    dat = fix(dat, addr)
  return [addr, 0, dat, alt]
 #Clarity
def create_brake_command(packer, apply_brake, pcm_override, pcm_cancel_cmd, chime, fcw, car_fingerprint, idx):
  # TODO: do we loose pressure if we keep pump off for long?
  commands = []  #Clarity
  pump_on = apply_brake > 0  #Clarity
  brakelights = apply_brake > 0
  brake_rq = apply_brake > 0
  pcm_fault_cmd = False

  #Clarity
  if car_fingerprint == CAR.CLARITY:
    bus = 2
    # This a bit of a hack but clarity brake msg flows into the last byte so
    # rather than change the fix() function just set accordingly here.
    apply_brake >>= 1
    if apply_brake & 1:
      idx += 0x8

  values = {
    "COMPUTER_BRAKE": apply_brake,
    "BRAKE_PUMP_REQUEST": pump_on,
    "CRUISE_OVERRIDE": pcm_override,
    "CRUISE_FAULT_CMD": pcm_fault_cmd,
    "CRUISE_CANCEL_CMD": pcm_cancel_cmd,
    "COMPUTER_BRAKE_REQUEST": brake_rq,
    "SET_ME_0X80": 0x80,
    "BRAKE_LIGHTS": brakelights,
    "CHIME": chime,
    # TODO: Why are there two bits for fcw? According to dbc file the first bit should also work
    "FCW": fcw << 1,
  }
  #Clarity
  #return packer.make_can_msg("BRAKE_COMMAND", 0, values, idx)
  commands.append(packer.make_can_msg("BRAKE_COMMAND", bus, values, idx))
  return commands

#Clarity
def create_gas_command(packer, gas_amount, idx):
  enable = gas_amount > 0.001
  values = {"ENABLE": enable}
  if enable:
    values["GAS_COMMAND"] = gas_amount * 255.
    values["GAS_COMMAND2"] = gas_amount * 255.
  return packer.make_can_msg("GAS_COMMAND", 0, values, idx)


def create_steering_control(packer, apply_steer, lkas_active, car_fingerprint, idx):
  values = {
    "STEER_TORQUE": apply_steer if lkas_active else 0,
    "STEER_TORQUE_REQUEST": lkas_active,
  }
  #Clarity
  # Set bus 2 for bosch or clarity.
  bus = 2 if car_fingerprint in HONDA_BOSCH or car_fingerprint == CAR.CLARITY else 0
  return packer.make_can_msg("STEERING_CONTROL", bus, values, idx)


def create_ui_commands(packer, pcm_speed, hud, car_fingerprint, is_metric, idx):
  commands = []
  bus = 0
  #Clarity
  if car_fingerprint == CAR.CLARITY:
    bus = 2

  # Bosch sends commands to bus 2.
  if car_fingerprint in HONDA_BOSCH:
    bus = 2
  else:
    acc_hud_values = {
      'PCM_SPEED': pcm_speed * CV.MS_TO_KPH,
      'PCM_GAS': hud.pcm_accel,
      'CRUISE_SPEED': hud.v_cruise,
      'ENABLE_MINI_CAR': hud.mini_car,
      'HUD_LEAD': hud.car,
      'HUD_DISTANCE': 3,    # max distance setting on display
      'IMPERIAL_UNIT': int(not is_metric),
      'SET_ME_X01_2': 1,
      'SET_ME_X01': 1,
    }
    #Clarity sends longitudinal control and UI messages to bus 0 and 2.
    commands.append(packer.make_can_msg("ACC_HUD", bus, acc_hud_values, idx))

  lkas_hud_values = {
    'SET_ME_X41': 0x41,
    'SET_ME_X48': 0x48,
    'STEERING_REQUIRED': hud.steer_required,
    'SOLID_LANES': hud.lanes,
    'BEEP': hud.beep,
  }
  #Clarity sends longitudinal control and UI messages to bus 0 and 2.
  if car_fingerprint == CAR.CLARITY:
    commands.append(packer.make_can_msg('LKAS_HUD', 2, lkas_hud_values, idx))

  if car_fingerprint in (CAR.CIVIC, CAR.ODYSSEY):

    radar_hud_values = {
      'ACC_ALERTS': hud.acc_alert,
      'LEAD_SPEED': 0x1fe,  # What are these magic values
      'LEAD_STATE': 0x7,
      'LEAD_DISTANCE': 0x1e,
    }
    commands.append(packer.make_can_msg('RADAR_HUD', 0, radar_hud_values, idx))
  return commands

#Clarity
def create_radar_commands(v_ego, car_fingerprint, new_radar_config, idx):
  """Creates an iterable of CAN messages for the radar system."""
  commands = []
  v_ego_kph = np.clip(int(round(v_ego * CV.MS_TO_KPH)), 0, 255)
  speed = struct.pack('!B', v_ego_kph)

  msg_0x300 = ("\xf9" + speed + "\x8a\xd0" +
               ("\x20" if idx == 0 or idx == 3 else "\x00") +
               "\x00\x00")
  msg_0x301 = VEHICLE_STATE_MSG[car_fingerprint]

  idx_0x300 = idx
  if car_fingerprint == CAR.CIVIC:
    idx_offset = 0xc if new_radar_config else 0x8   # radar in civic 2018 requires 0xc
    idx_0x300 += idx_offset

  commands.append(make_can_msg(0x300, msg_0x300, idx_0x300, 1))
  commands.append(make_can_msg(0x301, msg_0x301, idx, 1))
  return commands

def spam_buttons_command(packer, button_val, idx):
  values = {
    'CRUISE_BUTTONS': button_val,
    'CRUISE_SETTING': 0,
  }
  return packer.make_can_msg("SCM_BUTTONS", 0, values, idx)
