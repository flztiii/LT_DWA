from agent import Agent
from state import JointState


class Human(Agent):
    def __init__(self, config, section):
        super().__init__(config, section)

    def act(self, ob):
        """
        The state for human is its full state and all other agents' observable states
        :param ob:
        :return:
        """
        state = JointState(self.get_full_state(), ob)
        action = self.policy.predict(state)
        return action
