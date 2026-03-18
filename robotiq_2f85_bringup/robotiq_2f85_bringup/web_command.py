#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from dataclasses import dataclass
import math

GRIPPER_POSITION_COMMAND = 'gripper_pos'
START_GRASP_COMMAND = 'start_grasp'
STOP_GRASP_COMMAND = 'stop_grasp'
GO_HOME_COMMAND = 'go_home'
_SUPPORTED_COMMANDS = {
    GRIPPER_POSITION_COMMAND,
    START_GRASP_COMMAND,
    STOP_GRASP_COMMAND,
    GO_HOME_COMMAND,
}


@dataclass(frozen=True)
class WebCommand:
    kind: str
    value: float | None


def _parse_numeric_value(value_text: str, key: str) -> float:
    try:
        value = float(value_text)
    except ValueError as exc:
        raise ValueError(
            f'Invalid numeric value for {key!r}: {value_text!r}'
        ) from exc

    if not math.isfinite(value):
        raise ValueError(f'Non-finite value is not allowed for {key!r}')

    return value


def parse_web_command(text: str) -> WebCommand:
    stripped = str(text).strip()
    if not stripped:
        raise ValueError('web command is empty')

    if ':' not in stripped:
        if stripped in _SUPPORTED_COMMANDS:
            return WebCommand(kind=stripped, value=None)
        raise ValueError(f'Unsupported web command: {stripped!r}')

    key_text, value_text = stripped.split(':', 1)
    key = key_text.strip()
    if key not in _SUPPORTED_COMMANDS:
        raise ValueError(f'Unsupported web command key: {key!r}')
    if key != GRIPPER_POSITION_COMMAND:
        raise ValueError(
            f'Command {key!r} does not accept a numeric payload; '
            f'use {key!r} without ":"'
        )

    value = _parse_numeric_value(value_text.strip(), key)
    return WebCommand(kind=key, value=value)


def extract_gripper_percent(command: WebCommand) -> float | None:
    if command.kind != GRIPPER_POSITION_COMMAND:
        return None
    return command.value


def is_start_grasp(command: WebCommand) -> bool:
    return command.kind == START_GRASP_COMMAND


def is_stop_grasp(command: WebCommand) -> bool:
    return command.kind == STOP_GRASP_COMMAND


def is_go_home(command: WebCommand) -> bool:
    return command.kind == GO_HOME_COMMAND
