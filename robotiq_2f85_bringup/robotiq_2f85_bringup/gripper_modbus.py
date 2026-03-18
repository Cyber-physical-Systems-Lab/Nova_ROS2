from dataclasses import dataclass
import re

_REG_ACTION = 1000
_REG_POS = 1001
_REG_SPD_FRC = 1002
_FEEDBACK_REG_ADDR = 2000
_FEEDBACK_REG_COUNT = 3
_FEEDBACK_VAL_TYPE = 'U16'
_FEEDBACK_POLL_INTERVAL_S = 0.05
_FEEDBACK_TIMEOUT_S = 3.0
_ACT_GOTO = 2304
_JOINT_OPEN = 0.0
_JOINT_CLOSED = 0.7929
_GOBJ_TERMINAL_VALUES = frozenset({1, 2, 3})
_GSTA_VALID_VALUES = frozenset({0, 1, 3})
_PUBLIC_ACTION_NAME = '/gripper_position_controller/gripper_cmd'
_INTERNAL_ACTION_NAME = '/gripper_modbus_manager/gripper_cmd'


@dataclass(frozen=True)
class RobotiqFeedback:
    gobj: int
    gpo: int
    gflt: int
    gpr: int
    current: int
    joint_position: float


def joint_to_reg(joint_position: float) -> int:
    clamped = max(_JOINT_OPEN, min(_JOINT_CLOSED, joint_position))
    normalized = (clamped - _JOINT_OPEN) / (_JOINT_CLOSED - _JOINT_OPEN)
    return int(round(normalized * 255))


def reg_to_joint(reg_value: int) -> float:
    clamped = max(0, min(255, int(reg_value)))
    normalized = clamped / 255.0
    return _JOINT_OPEN + normalized * (_JOINT_CLOSED - _JOINT_OPEN)


def parse_register_payload(payload: str) -> list[int]:
    tokens = [token for token in re.split(r'[\s,]+', payload.strip()) if token]
    if len(tokens) < _FEEDBACK_REG_COUNT:
        raise ValueError(
            f'Expected at least {_FEEDBACK_REG_COUNT} register values, '
            f'got {len(tokens)} from {payload!r}'
        )

    values = []
    for token in tokens:
        try:
            values.append(int(token, 0) & 0xFFFF)
        except ValueError as exc:
            raise ValueError(
                f'Invalid register token {token!r} in payload {payload!r}'
            ) from exc
    return values


def decode_feedback_registers(registers: list[int]) -> RobotiqFeedback:
    if len(registers) < _FEEDBACK_REG_COUNT:
        raise ValueError(
            f'Expected {_FEEDBACK_REG_COUNT} registers, got {len(registers)}'
        )

    status_reg, fault_req_reg, pos_current_reg = registers[:_FEEDBACK_REG_COUNT]

    status_byte = (status_reg >> 8) & 0xFF
    gflt = (fault_req_reg >> 8) & 0xFF
    gpr = fault_req_reg & 0xFF
    gpo = (pos_current_reg >> 8) & 0xFF
    current = pos_current_reg & 0xFF
    gobj = (status_byte >> 6) & 0x03

    return RobotiqFeedback(
        gobj=gobj,
        gpo=gpo,
        gflt=gflt,
        gpr=gpr,
        current=current,
        joint_position=reg_to_joint(gpo),
    )


def feedback_registers_look_like_robotiq(registers: list[int]) -> bool:
    if len(registers) < _FEEDBACK_REG_COUNT:
        return False

    status_reg, fault_req_reg, _ = registers[:_FEEDBACK_REG_COUNT]
    status_byte = (status_reg >> 8) & 0xFF
    status_low_byte = status_reg & 0xFF
    gact = status_byte & 0x01
    gsta = (status_byte >> 4) & 0x03
    gflt = (fault_req_reg >> 8) & 0xFF

    if status_low_byte != 0:
        return False
    if (status_byte & 0x06) != 0:
        return False
    if status_byte == 0:
        return False
    if gact != 1:
        return False
    if gsta not in _GSTA_VALID_VALUES:
        return False
    if gflt > 0x0F:
        return False

    return True


def feedback_is_terminal(feedback: RobotiqFeedback) -> bool:
    return feedback.gobj in _GOBJ_TERMINAL_VALUES
