# Copyright 2015 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.


class TestMsgs:
    """Test Roast Messages."""

    def test_msg_imports(self):
        """Test that all messages can be imported."""
        from roast_interfaces.msg import (  # pylint: disable=unused-import, import-outside-toplevel # noqa: E501, F401
            Diagnostics,
            Robot,
            Vision,
        )


class TestSrvs:
    """Test Roast Services."""

    def test_srv_imports(self):
        """Test that all services can be imported."""
        from roast_interfaces.srv import (  # pylint: disable=unused-import, import-outside-toplevel # noqa: E501, F401
            Fan,
            JetsonClocks,
            NVPModel,
        )


class TestActions:
    """Test Roast Actions."""

    def test_action_imports(self):
        """Test that all actions can be imported."""
        from roast_interfaces.action import (  # pylint: disable=unused-import, import-outside-toplevel # noqa: E501, F401
            ConnectToChargingStation,
            GoToPose,
            PatrolAction,
            TrackTarget,
        )
