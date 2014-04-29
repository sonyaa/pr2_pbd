"""Class that holds distribution information for all actions"""
from active_learning.ArmStepDistribution import ArmStepDistribution
from step_types.Action import Action


class ActionDistribution:
    def __init__(self):
        self._action_step_distributions = []
        self._n_actions = 0
        self._n_frames = 0
        self._cur_n_frames = 0

    #def new_action(self):
    #    self._n_actions += 1
    #    # When second action is created, remember the number of key frames in the first action - that's the reference.
    #    if self._n_actions == 2:
    #        self._n_frames = self._cur_n_frames
    #    self._cur_n_frames = 0
    #
    #def add_action_step(self, action_step):
    #    # If we're recording the first action, create the distribution. Otherwise, just add to it.
    #    if self._n_actions == 1:
    #        self._action_step_distributions.append(ActionStepDistribution(self._cur_n_frames))
    #    elif self._cur_n_frames == self._n_frames:
    #        # If this action has more frames than the first action, ignore the extra frames.
    #        return
    #    self._action_step_distributions[self._cur_n_frames].add_action_step(action_step)
    #    self._cur_n_frames += 1

    def add_action(self, action):
        if self._n_actions == 0:
            self._n_frames = action.n_frames()
            for i in range(self._n_frames):
                self._action_step_distributions.append(ArmStepDistribution(i))
        for i in range(self._n_frames):
            self._action_step_distributions[i].add_action_step(action.get_step(i))
        self._n_actions += 1

    def update_viz(self):
        for i in range(self._n_frames):
            self._action_step_distributions[i].update_viz()


    def get_generated_action(self, object_list):
        generated_action = Action(-1, None)
        for i in range(self._n_frames):
            step_distribution = self._action_step_distributions[i]
            new_step = step_distribution.get_sampled_action_step()
            generated_action.add_action_step(new_step, object_list)
        return generated_action