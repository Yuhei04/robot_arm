#include <Dynamixel2Arduino.h>

#define DXL_SERIAL Serial3
#define DEBUG_SERIAL Serial

const int DXL_DIR_PIN = 84;
const float DXL_PROTOCOL_VERSION = 2.0;
const uint32_t DXL_BAUDRATE = 57600;

const uint8_t DXL_IDS[] = {1, 2, 3, 4, 5, 6};
const size_t DXL_COUNT = sizeof(DXL_IDS) / sizeof(DXL_IDS[0]);
const int32_t WIGGLE_RAW = 80;
const uint8_t TEST_BASE_ID = 1;
const float DXL_TICKS_PER_REV = 4096.0f;
const float MAX_SINGLE_MOVE_DEG = 180.0f;
const float ZERO_TOLERANCE_DEG = 0.2f;
const uint32_t ZERO_STEP_DELAY_MS = 700;
const int32_t SAFE_PROFILE_VELOCITY = 80;
const int32_t SAFE_PROFILE_ACCELERATION = 20;
const int32_t DXL_ZERO_RAW[] = {2048, 2048, 2048, 2048, 2048, 2048};
const float JOINT_LIMIT_MIN_DEG[] = {-90.0f, -90.0f, -90.0f, -90.0f, -90.0f, -90.0f};
const float JOINT_LIMIT_MAX_DEG[] = {90.0f, 90.0f, 90.0f, 90.0f, 90.0f, 90.0f};

Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);

bool jointPositionPrepared[DXL_COUNT] = {false, false, false, false, false, false};
bool haveLastCommandDeg = false;
float lastCommandDeg[DXL_COUNT] = {0, 0, 0, 0, 0, 0};

void resetPreparedJoints() {
  for (size_t i = 0; i < DXL_COUNT; i++) {
    jointPositionPrepared[i] = false;
  }
  haveLastCommandDeg = false;
}

void printHelp() {
  DEBUG_SERIAL.println();
  DEBUG_SERIAL.println("Commands:");
  DEBUG_SERIAL.println("  s: ping scan");
  DEBUG_SERIAL.println("  r: read present positions");
  DEBUG_SERIAL.println("  o: print fixed joint zero raw positions");
  DEBUG_SERIAL.println("  a: read joint angles from fixed DYNAMIXEL zero");
  DEBUG_SERIAL.println("  l: print joint limits");
  DEBUG_SERIAL.println("  e: torque enable");
  DEBUG_SERIAL.println("  d: torque disable");
  DEBUG_SERIAL.println("  j <id> <delta_deg>: move one joint relative to current angle");
  DEBUG_SERIAL.println("  m <id> <target_deg>: move one joint to angle from fixed zero");
  DEBUG_SERIAL.println("  q <j1_deg> <j2_deg> <j3_deg> <j4_deg> <j5_deg> <j6_deg>: move all joints together");
  DEBUG_SERIAL.println("  0: move all joints to 0 deg step by step");
  DEBUG_SERIAL.println("  w: small wiggle around current position");
  DEBUG_SERIAL.println("  g: lower ID1 position P gain for hunting test");
  DEBUG_SERIAL.println("  p: print ID1 position PID gains");
  DEBUG_SERIAL.println();
}

int findJointIndex(uint8_t id) {
  for (size_t i = 0; i < DXL_COUNT; i++) {
    if (DXL_IDS[i] == id) {
      return i;
    }
  }
  return -1;
}

int32_t degToRawOffset(float deg) {
  float raw = deg * DXL_TICKS_PER_REV / 360.0f;
  if (raw >= 0.0f) {
    return (int32_t)(raw + 0.5f);
  }
  return (int32_t)(raw - 0.5f);
}

int32_t normalizeRawOffset(int32_t rawOffset) {
  while (rawOffset > 2047) {
    rawOffset -= 4096;
  }
  while (rawOffset < -2048) {
    rawOffset += 4096;
  }
  return rawOffset;
}

int32_t wrapRawPosition(int32_t rawPosition) {
  while (rawPosition >= 4096) {
    rawPosition -= 4096;
  }
  while (rawPosition < 0) {
    rawPosition += 4096;
  }
  return rawPosition;
}

float rawOffsetToDeg(int32_t rawOffset) {
  return (float)normalizeRawOffset(rawOffset) * 360.0f / DXL_TICKS_PER_REV;
}

void printJointLimits() {
  DEBUG_SERIAL.println("Joint limits from fixed zero:");
  for (size_t i = 0; i < DXL_COUNT; i++) {
    DEBUG_SERIAL.print("ID ");
    DEBUG_SERIAL.print(DXL_IDS[i]);
    DEBUG_SERIAL.print(": ");
    DEBUG_SERIAL.print(JOINT_LIMIT_MIN_DEG[i], 1);
    DEBUG_SERIAL.print(" deg to ");
    DEBUG_SERIAL.print(JOINT_LIMIT_MAX_DEG[i], 1);
    DEBUG_SERIAL.println(" deg");
  }
  DEBUG_SERIAL.print("Max single move: ");
  DEBUG_SERIAL.print(MAX_SINGLE_MOVE_DEG, 1);
  DEBUG_SERIAL.println(" deg");
  DEBUG_SERIAL.print("Safe profile velocity: ");
  DEBUG_SERIAL.println(SAFE_PROFILE_VELOCITY);
  DEBUG_SERIAL.print("Safe profile acceleration: ");
  DEBUG_SERIAL.println(SAFE_PROFILE_ACCELERATION);
}

void printFixedZeroPositions() {
  DEBUG_SERIAL.println("Fixed joint zero raw positions:");
  for (size_t i = 0; i < DXL_COUNT; i++) {
    DEBUG_SERIAL.print("ID ");
    DEBUG_SERIAL.print(DXL_IDS[i]);
    DEBUG_SERIAL.print(": ");
    DEBUG_SERIAL.println(DXL_ZERO_RAW[i]);
  }
}

void scanIds() {
  DEBUG_SERIAL.println("Scanning DYNAMIXEL IDs...");
  for (size_t i = 0; i < DXL_COUNT; i++) {
    uint8_t id = DXL_IDS[i];
    if (dxl.ping(id)) {
      DEBUG_SERIAL.print("ID ");
      DEBUG_SERIAL.print(id);
      DEBUG_SERIAL.println(": OK");
    } else {
      DEBUG_SERIAL.print("ID ");
      DEBUG_SERIAL.print(id);
      DEBUG_SERIAL.println(": not found");
    }
  }
}

void readJointAngles() {
  DEBUG_SERIAL.println("Reading joint angles from fixed DYNAMIXEL zero...");
  for (size_t i = 0; i < DXL_COUNT; i++) {
    uint8_t id = DXL_IDS[i];
    if (!dxl.ping(id)) {
      DEBUG_SERIAL.print("ID ");
      DEBUG_SERIAL.print(id);
      DEBUG_SERIAL.println(": not found");
      continue;
    }

    int32_t position = dxl.getPresentPosition(id);
    float angleDeg = rawOffsetToDeg(position - DXL_ZERO_RAW[i]);
    DEBUG_SERIAL.print("ID ");
    DEBUG_SERIAL.print(id);
    DEBUG_SERIAL.print(": ");
    DEBUG_SERIAL.print(angleDeg, 2);
    DEBUG_SERIAL.println(" deg");
  }
}

void readPositions() {
  DEBUG_SERIAL.println("Reading present positions...");
  for (size_t i = 0; i < DXL_COUNT; i++) {
    uint8_t id = DXL_IDS[i];
    if (!dxl.ping(id)) {
      DEBUG_SERIAL.print("ID ");
      DEBUG_SERIAL.print(id);
      DEBUG_SERIAL.println(": not found");
      continue;
    }

    int32_t position = dxl.getPresentPosition(id);
    DEBUG_SERIAL.print("ID ");
    DEBUG_SERIAL.print(id);
    DEBUG_SERIAL.print(": ");
    DEBUG_SERIAL.println(position);
  }
}

bool isSafeTargetDeg(int jointIndex, float targetDeg) {
  return targetDeg >= JOINT_LIMIT_MIN_DEG[jointIndex] && targetDeg <= JOINT_LIMIT_MAX_DEG[jointIndex];
}

float getJointAngleDeg(int jointIndex) {
  uint8_t id = DXL_IDS[jointIndex];
  int32_t position = dxl.getPresentPosition(id);
  return rawOffsetToDeg(position - DXL_ZERO_RAW[jointIndex]);
}

void prepareJointForPositionMove(uint8_t id, int32_t holdRaw) {
  int jointIndex = findJointIndex(id);
  if (jointIndex < 0) {
    return;
  }
  if (jointPositionPrepared[jointIndex]) {
    return;
  }

  dxl.torqueOff(id);
  dxl.setOperatingMode(id, OP_POSITION);
  dxl.writeControlTableItem(ControlTableItem::PROFILE_VELOCITY, id, SAFE_PROFILE_VELOCITY);
  dxl.writeControlTableItem(ControlTableItem::PROFILE_ACCELERATION, id, SAFE_PROFILE_ACCELERATION);
  dxl.setGoalPosition(id, holdRaw);
  dxl.torqueOn(id);
  jointPositionPrepared[jointIndex] = true;
}

void moveJointToAngle(uint8_t id, float targetDeg) {
  int jointIndex = findJointIndex(id);
  if (jointIndex < 0) {
    DEBUG_SERIAL.println("Unknown DYNAMIXEL ID.");
    return;
  }
  if (!dxl.ping(id)) {
    DEBUG_SERIAL.println("DYNAMIXEL ID not found.");
    return;
  }
  if (!isSafeTargetDeg(jointIndex, targetDeg)) {
    DEBUG_SERIAL.println("Rejected: target angle is outside the relative joint limit.");
    return;
  }

  int32_t presentRaw = dxl.getPresentPosition(id);
  float currentDeg = rawOffsetToDeg(presentRaw - DXL_ZERO_RAW[jointIndex]);
  float deltaDeg = targetDeg - currentDeg;
  if (deltaDeg > MAX_SINGLE_MOVE_DEG || deltaDeg < -MAX_SINGLE_MOVE_DEG) {
    DEBUG_SERIAL.println("Rejected: requested move is larger than max single move.");
    return;
  }

  int32_t targetRaw = wrapRawPosition(DXL_ZERO_RAW[jointIndex] + degToRawOffset(targetDeg));
  prepareJointForPositionMove(id, presentRaw);
  delay(50);
  dxl.setGoalPosition(id, targetRaw);

  DEBUG_SERIAL.print("ID ");
  DEBUG_SERIAL.print(id);
  DEBUG_SERIAL.print(": ");
  DEBUG_SERIAL.print(currentDeg, 2);
  DEBUG_SERIAL.print(" deg -> ");
  DEBUG_SERIAL.print(targetDeg, 2);
  DEBUG_SERIAL.print(" deg, raw ");
  DEBUG_SERIAL.println(targetRaw);
}

void moveAllJointsToAngles(float targetDegs[]) {
  int32_t presentRaw[DXL_COUNT];
  int32_t targetRaw[DXL_COUNT];
  float currentDeg[DXL_COUNT];

  for (size_t i = 0; i < DXL_COUNT; i++) {
    uint8_t id = DXL_IDS[i];
    if (!jointPositionPrepared[i] && !dxl.ping(id)) {
      DEBUG_SERIAL.print("Rejected: ID ");
      DEBUG_SERIAL.print(id);
      DEBUG_SERIAL.println(" not found");
      return;
    }
    if (!isSafeTargetDeg(i, targetDegs[i])) {
      DEBUG_SERIAL.print("Rejected: ID ");
      DEBUG_SERIAL.print(id);
      DEBUG_SERIAL.println(" target is outside the relative joint limit.");
      return;
    }

    if (haveLastCommandDeg && jointPositionPrepared[i]) {
      currentDeg[i] = lastCommandDeg[i];
      presentRaw[i] = wrapRawPosition(DXL_ZERO_RAW[i] + degToRawOffset(currentDeg[i]));
    } else {
      presentRaw[i] = dxl.getPresentPosition(id);
      currentDeg[i] = rawOffsetToDeg(presentRaw[i] - DXL_ZERO_RAW[i]);
    }
    float deltaDeg = targetDegs[i] - currentDeg[i];
    if (deltaDeg > MAX_SINGLE_MOVE_DEG || deltaDeg < -MAX_SINGLE_MOVE_DEG) {
      DEBUG_SERIAL.print("Rejected: ID ");
      DEBUG_SERIAL.print(id);
      DEBUG_SERIAL.println(" requested move is larger than max single move.");
      return;
    }
    targetRaw[i] = wrapRawPosition(DXL_ZERO_RAW[i] + degToRawOffset(targetDegs[i]));
  }

  bool preparedThisCall = false;
  for (size_t i = 0; i < DXL_COUNT; i++) {
    if (!jointPositionPrepared[i]) {
      preparedThisCall = true;
    }
    prepareJointForPositionMove(DXL_IDS[i], presentRaw[i]);
  }
  if (preparedThisCall) {
    delay(20);
  }
  for (size_t i = 0; i < DXL_COUNT; i++) {
    dxl.setGoalPosition(DXL_IDS[i], targetRaw[i]);
  }

  for (size_t i = 0; i < DXL_COUNT; i++) {
    lastCommandDeg[i] = targetDegs[i];
  }
  haveLastCommandDeg = true;
}

void moveJointToZeroStepped(uint8_t id) {
  int jointIndex = findJointIndex(id);
  if (jointIndex < 0) {
    DEBUG_SERIAL.println("Unknown DYNAMIXEL ID.");
    return;
  }
  if (!dxl.ping(id)) {
    DEBUG_SERIAL.print("ID ");
    DEBUG_SERIAL.print(id);
    DEBUG_SERIAL.println(": not found");
    return;
  }

  float currentDeg = getJointAngleDeg(jointIndex);
  if (!isSafeTargetDeg(jointIndex, currentDeg)) {
    DEBUG_SERIAL.print("ID ");
    DEBUG_SERIAL.print(id);
    DEBUG_SERIAL.println(": rejected because current angle is outside the joint limit.");
    return;
  }

  int32_t presentRaw = dxl.getPresentPosition(id);
  prepareJointForPositionMove(id, presentRaw);
  delay(50);

  DEBUG_SERIAL.print("ID ");
  DEBUG_SERIAL.print(id);
  DEBUG_SERIAL.print(": moving ");
  DEBUG_SERIAL.print(currentDeg, 2);
  DEBUG_SERIAL.println(" deg -> 0.00 deg");

  for (size_t step = 0; step < 20; step++) {
    currentDeg = getJointAngleDeg(jointIndex);
    if (currentDeg <= ZERO_TOLERANCE_DEG && currentDeg >= -ZERO_TOLERANCE_DEG) {
      break;
    }

    float nextDeg = 0.0f;
    if (currentDeg > MAX_SINGLE_MOVE_DEG) {
      nextDeg = currentDeg - MAX_SINGLE_MOVE_DEG;
    } else if (currentDeg < -MAX_SINGLE_MOVE_DEG) {
      nextDeg = currentDeg + MAX_SINGLE_MOVE_DEG;
    }

    if (!isSafeTargetDeg(jointIndex, nextDeg)) {
      DEBUG_SERIAL.print("ID ");
      DEBUG_SERIAL.print(id);
      DEBUG_SERIAL.println(": stopped because next target is outside the joint limit.");
      return;
    }

    int32_t targetRaw = wrapRawPosition(DXL_ZERO_RAW[jointIndex] + degToRawOffset(nextDeg));
    dxl.setGoalPosition(id, targetRaw);

    DEBUG_SERIAL.print("ID ");
    DEBUG_SERIAL.print(id);
    DEBUG_SERIAL.print(": target ");
    DEBUG_SERIAL.print(nextDeg, 2);
    DEBUG_SERIAL.print(" deg, raw ");
    DEBUG_SERIAL.println(targetRaw);
    delay(ZERO_STEP_DELAY_MS);
  }

  int32_t targetRaw = wrapRawPosition(DXL_ZERO_RAW[jointIndex]);
  dxl.setGoalPosition(id, targetRaw);
  DEBUG_SERIAL.print("ID ");
  DEBUG_SERIAL.print(id);
  DEBUG_SERIAL.print(": final target 0.00 deg, raw ");
  DEBUG_SERIAL.println(targetRaw);
  delay(ZERO_STEP_DELAY_MS);
}

void moveAllJointsToZero() {
  DEBUG_SERIAL.println("Moving all joints to 0 deg step by step...");
  for (size_t i = 0; i < DXL_COUNT; i++) {
    moveJointToZeroStepped(DXL_IDS[i]);
  }
  DEBUG_SERIAL.println("All-zero command finished.");
}

void moveJointByAngle(uint8_t id, float deltaDeg) {
  if (deltaDeg > MAX_SINGLE_MOVE_DEG || deltaDeg < -MAX_SINGLE_MOVE_DEG) {
    DEBUG_SERIAL.println("Rejected: delta is larger than max single move.");
    return;
  }

  int jointIndex = findJointIndex(id);
  if (jointIndex < 0) {
    DEBUG_SERIAL.println("Unknown DYNAMIXEL ID.");
    return;
  }
  if (!dxl.ping(id)) {
    DEBUG_SERIAL.println("DYNAMIXEL ID not found.");
    return;
  }

  int32_t presentRaw = dxl.getPresentPosition(id);
  float currentDeg = rawOffsetToDeg(presentRaw - DXL_ZERO_RAW[jointIndex]);
  moveJointToAngle(id, currentDeg + deltaDeg);
}

void setTorque(bool enabled) {
  for (size_t i = 0; i < DXL_COUNT; i++) {
    uint8_t id = DXL_IDS[i];
    if (!dxl.ping(id)) {
      continue;
    }

    if (enabled) {
      int32_t position = dxl.getPresentPosition(id);
      dxl.setGoalPosition(id, position);
      dxl.torqueOn(id);
    } else {
      dxl.torqueOff(id);
      jointPositionPrepared[i] = false;
    }

    DEBUG_SERIAL.print("ID ");
    DEBUG_SERIAL.print(id);
    DEBUG_SERIAL.println(enabled ? ": torque on" : ": torque off");
    if (!enabled) {
      haveLastCommandDeg = false;
    }
  }
}

void wiggle() {
  DEBUG_SERIAL.println("Wiggle test: current -> +80 -> current -> -80 -> current");
  for (size_t i = 0; i < DXL_COUNT; i++) {
    uint8_t id = DXL_IDS[i];
    if (!dxl.ping(id)) {
      continue;
    }

    dxl.torqueOff(id);
    dxl.setOperatingMode(id, OP_POSITION);
    dxl.torqueOn(id);

    int32_t center = dxl.getPresentPosition(id);

    DEBUG_SERIAL.print("ID ");
    DEBUG_SERIAL.print(id);
    DEBUG_SERIAL.print(": center ");
    DEBUG_SERIAL.println(center);

    dxl.setGoalPosition(id, center + WIGGLE_RAW);
    delay(700);
    dxl.setGoalPosition(id, center);
    delay(700);
    dxl.setGoalPosition(id, center - WIGGLE_RAW);
    delay(700);
    dxl.setGoalPosition(id, center);
    delay(700);
  }
}

void printBaseGains() {
  uint8_t id = TEST_BASE_ID;
  if (!dxl.ping(id)) {
    DEBUG_SERIAL.println("ID 1: not found");
    return;
  }

  DEBUG_SERIAL.print("ID 1 Position D Gain: ");
  DEBUG_SERIAL.println(dxl.readControlTableItem(ControlTableItem::POSITION_D_GAIN, id));
  DEBUG_SERIAL.print("ID 1 Position I Gain: ");
  DEBUG_SERIAL.println(dxl.readControlTableItem(ControlTableItem::POSITION_I_GAIN, id));
  DEBUG_SERIAL.print("ID 1 Position P Gain: ");
  DEBUG_SERIAL.println(dxl.readControlTableItem(ControlTableItem::POSITION_P_GAIN, id));
}

void lowerBasePGain() {
  uint8_t id = TEST_BASE_ID;
  if (!dxl.ping(id)) {
    DEBUG_SERIAL.println("ID 1: not found");
    return;
  }

  dxl.torqueOff(id);
  jointPositionPrepared[findJointIndex(id)] = false;
  dxl.writeControlTableItem(ControlTableItem::POSITION_P_GAIN, id, 400);
  dxl.writeControlTableItem(ControlTableItem::POSITION_I_GAIN, id, 0);
  dxl.writeControlTableItem(ControlTableItem::POSITION_D_GAIN, id, 0);

  int32_t position = dxl.getPresentPosition(id);
  dxl.setGoalPosition(id, position);
  dxl.torqueOn(id);

  DEBUG_SERIAL.println("ID 1: set P=400, I=0, D=0 and torque on");
  printBaseGains();
}

void setup() {
  DEBUG_SERIAL.begin(115200);
  DEBUG_SERIAL.setTimeout(2000);
  while (!DEBUG_SERIAL) {
    delay(10);
  }

  dxl.begin(DXL_BAUDRATE);
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);

  DEBUG_SERIAL.println("OpenCR DYNAMIXEL check");
  DEBUG_SERIAL.print("Baudrate: ");
  DEBUG_SERIAL.println(DXL_BAUDRATE);
  printHelp();
}

void loop() {
  if (!DEBUG_SERIAL.available()) {
    return;
  }

  char command = DEBUG_SERIAL.read();
  switch (command) {
    case 's':
      scanIds();
      break;
    case 'r':
      readPositions();
      break;
    case 'o':
      printFixedZeroPositions();
      break;
    case 'a':
      readJointAngles();
      break;
    case 'l':
      printJointLimits();
      break;
    case 'e':
      setTorque(true);
      break;
    case 'd':
      setTorque(false);
      break;
    case 'j': {
      int id = DEBUG_SERIAL.parseInt();
      float deltaDeg = DEBUG_SERIAL.parseFloat();
      moveJointByAngle((uint8_t)id, deltaDeg);
      break;
    }
    case 'm': {
      int id = DEBUG_SERIAL.parseInt();
      float targetDeg = DEBUG_SERIAL.parseFloat();
      moveJointToAngle((uint8_t)id, targetDeg);
      break;
    }
    case 'q': {
      float targets[DXL_COUNT];
      for (size_t i = 0; i < DXL_COUNT; i++) {
        targets[i] = DEBUG_SERIAL.parseFloat();
      }
      moveAllJointsToAngles(targets);
      break;
    }
    case '0':
      moveAllJointsToZero();
      break;
    case 'w':
      wiggle();
      break;
    case 'g':
      lowerBasePGain();
      break;
    case 'p':
      printBaseGains();
      break;
    case 'h':
    case '?':
      printHelp();
      break;
    default:
      break;
  }
}
