"""Common accessors for the separate plane and frame instruction messages."""

import assembly_manager_interfaces.msg as ami_msg
from typing import Union


AssemblyInstructionType = Union[
    ami_msg.AssemblyInstruction, ami_msg.FrameAssemblyInstruction,
]


def instruction_components(instruction):
    """Return the two display components and whether the first one moves."""
    if isinstance(instruction, ami_msg.FrameAssemblyInstruction):
        return (
            instruction.frame_match.assembly_frame.component,
            instruction.frame_match.target_frame.component,
            True,
        )
    return (
        instruction.component_1,
        instruction.component_2,
        instruction.component_1_is_moving_part,
    )


def moving_and_target_components(instruction):
    """Return moving and stationary component names for either instruction."""
    first, second, first_moves = instruction_components(instruction)
    return (first, second) if first_moves else (second, first)
