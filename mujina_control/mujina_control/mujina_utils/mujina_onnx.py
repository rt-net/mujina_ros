# The MIT License (MIT)
#
# Copyright (c) 2026 RT Corporation
#
# Permission is hereby granted, free of charge, to any person obtaining a copy of
# this software and associated documentation files (the "Software"), to deal in
# the Software without restriction, including without limitation the rights to
# use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
# the Software, and to permit persons to whom the Software is furnished to do so,
# subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
# FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
# COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
# IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
# CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

import os

import onnxruntime
import torch
from ament_index_python.packages import get_package_share_directory

from mujina_control.mujina_utils import (
    P,
    get_policy_observation,
    normalization,
)


class OnnxPredictor:
    def __init__(self, model_path: str) -> None:
        self.session = onnxruntime.InferenceSession(
            model_path, providers=["CPUExecutionProvider"]
        )
        self.input_name = self.session.get_inputs()[0].name
        self.output_name = self.session.get_outputs()[0].name

    def get_onnx_output(self, obs: torch.Tensor) -> torch.Tensor:
        clip_obs = normalization.clip_observations
        obs = torch.clip(obs, -clip_obs, clip_obs)

        with torch.no_grad():
            actions: torch.Tensor = torch.from_numpy(
                self.session.run(
                    [self.output_name], {self.input_name: obs.detach().numpy().copy()}
                )[0]
            ).clone()

        clip_actions = normalization.clip_actions
        actions = torch.clip(actions, -clip_actions, clip_actions)
        return actions[0]  # reference angle [rad]


def test_read_onnx_policy():
    model_path = os.path.join(
        get_package_share_directory("mujina_control"),
        "models/policy.onnx",
    )
    onnx_predictor = OnnxPredictor(model_path)
    dummy_obs = torch.zeros((1, 45), dtype=torch.float32)
    output = onnx_predictor.get_onnx_output(dummy_obs)
    print(output)


def test_get_policy_output():
    model_path = os.path.join(
        get_package_share_directory("mujina_control"),
        "models/policy.onnx",
    )
    onnx_predictor = OnnxPredictor(model_path)
    base_quat = [0.0, 0.0, 0.0, 1.0]
    base_ang_vel = [0.0, 0.0, 0.0]
    command = [0.0, 0.0, 0.0, 0.0]
    dof_pos = P.DEFAULT_ANGLE
    dof_vel = [0] * 12
    actions = [0] * 12
    obs = get_policy_observation(
        base_quat,
        base_ang_vel,
        command,
        dof_pos,
        dof_vel,
        actions,
    )
    print(obs.numpy()[0])
    print(onnx_predictor.get_onnx_output(obs))


if __name__ == "__main__":
    print("# test_read_onnx_policy")
    test_read_onnx_policy()
    print("# test_get_policy_output")
    test_get_policy_output()
