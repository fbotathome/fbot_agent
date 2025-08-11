from yasmin import StateMachine, Blackboard, CbState
from yasmin_ros.basic_outcomes import SUCCEED, CANCEL, ABORT
from state_machine.machines import SaySomethingMachine, RivaListenSomethingMachine

from smolagents import tool

@tool
def say_something(text: str) -> bool:
    """
    Allows you to say something using the robot's speech system.

    Args:
        text: The text to be spoken by the robot.
    
    Returns:
        bool: True if the speech was successfully initiated, False otherwise.
    """
    sm = StateMachine(outcomes=['aborted', 'canceled', 'succeeded', 'timeout'])
    sm.add_state(
        name='SAY_SOMETHING',
        state=CbState(outcomes=[SUCCEED], cb=lambda blackboard: SUCCEED), #SaySomethingMachine(data=text),
        transitions={
            SUCCEED: SUCCEED,
            # ABORT: ABORT,
            # CANCEL: CANCEL,
            # TIMEOUT: TIMEOUT
        }
    )
    
    blackboard = Blackboard()
    outcome = sm.execute(blackboard=blackboard)
    return outcome == SUCCEED

@tool
def listen_something(boosted_words: list[str] = []) -> str:
    """
    Listens for a specific command or phrase using the robot's speech recognition system.

    Args:
        boosted_words (list[str], optional): A list of words or phrases to boost recognition for.
        It is useful for improving the accuracy of recognizing specific terms that you expect to hear.

    Returns:
        str: The recognized text. It will be an empty string if nothing was recognized.
    """
    sm = StateMachine(outcomes=['aborted', 'canceled', 'succeeded', 'timeout'])
    sm.add_state(
        name='LISTEN_SOMETHING',
        state=CbState(outcomes=[SUCCEED], cb=lambda blackboard: SUCCEED), #RivaListenSomethingMachine(boosted_words=boosted_words),
        transitions={
            SUCCEED: SUCCEED,
            # ABORT: ABORT,
            # CANCEL: CANCEL,
            # TIMEOUT: TIMEOUT
        }
    )

    blackboard = Blackboard()
    outcome = sm.execute(blackboard=blackboard)
    text = blackboard['_text'] if '_text' in blackboard else 'Whats the capital of Bahia?'
    return text