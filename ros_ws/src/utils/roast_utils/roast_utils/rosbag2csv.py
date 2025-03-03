import os
import sqlite3
import sys

import yaml
from PyQt5 import QtCore, QtWidgets
from rclpy.serialization import deserialize_message
from rosidl_runtime_py import message_to_csv
from rosidl_runtime_py.utilities import get_message

from roast.glogging import Logger

LOG = Logger("rosbag2csv", _level=1)


class SimplePyQtGUIKit:
    def QuitApp(self):
        QtWidgets.QApplication.quit()

    @classmethod
    def GetFilePath(self, caption="Open File", filefilter="", isApp=False):
        """
        "Images (*.png *.xpm *.jpg);;Text files (*.txt);;XML files (*.xml)"
        """

        if not isApp:
            QtWidgets.QApplication(sys.argv)
        files = QtWidgets.QFileDialog.getOpenFileNames(
            caption=caption, filter=filefilter
        )

        strlist = []
        for file in files:
            if type(file) == list:
                for f in file:
                    strlist.append(str(f))
            else:
                strlist.append(str(file))
        return strlist

    @classmethod
    def GetCheckButtonSelect(self, selectList, title="Select", msg="", app=None):
        """
        Get selected check button options

        title: Window name
        mag: Label of the check button
        return selected dictionary
            {'sample b': False, 'sample c': False, 'sample a': False}
        """

        if app is None:
            app = QtWidgets.QApplication(sys.argv)
        win = QtWidgets.QWidget()
        scrollArea = QtWidgets.QScrollArea()
        scrollArea.setWidgetResizable(True)
        scrollAreaWidgetContents = QtWidgets.QWidget(scrollArea)
        scrollAreaWidgetContents.setGeometry(QtCore.QRect(0, 0, 380, 247))
        scrollArea.setWidget(scrollAreaWidgetContents)
        layout = QtWidgets.QGridLayout()
        verticalLayoutScroll = QtWidgets.QVBoxLayout(scrollAreaWidgetContents)
        layoutIndex = 0

        if msg != "":
            label = QtWidgets.QLabel(msg)
            layout.addWidget(label, layoutIndex, 0)
            layoutIndex = layoutIndex + 1

        checkboxs = []
        for select in selectList:
            checkbox = QtWidgets.QCheckBox(select)
            verticalLayoutScroll.addWidget(checkbox)
            layoutIndex = layoutIndex + 1
            checkboxs.append(checkbox)

        layout.addWidget(scrollArea)
        btn = QtWidgets.QPushButton("OK")
        btn.clicked.connect(app.quit)
        layout.addWidget(btn, layoutIndex, 0)
        layoutIndex = layoutIndex + 1

        win.setLayout(layout)
        win.setWindowTitle(title)
        win.show()
        app.exec_()

        result = {}
        for checkbox, select in zip(checkboxs, selectList):
            result[select] = checkbox.isChecked()

        return result


class BagFileParser:
    def __init__(self, bag_file):
        self.conn = sqlite3.connect(bag_file)
        self.cursor = self.conn.cursor()

        ## create a message type map
        topics_data = self.cursor.execute(
            "SELECT id, name, type FROM topics"
        ).fetchall()
        self.topic_type = {name_of: type_of for _, name_of, type_of in topics_data}
        self.topic_id = {name_of: id_of for id_of, name_of, _ in topics_data}
        self.topic_msg_message = {
            name_of: get_message(type_of) for _, name_of, type_of in topics_data
        }

    def __del__(self):
        self.conn.close()

    # Return [(timestamp0, message0), (timestamp1, message1), ...]
    def get_messages(self, topic_name, convert_to_csv=False):
        topic_id = self.topic_id[topic_name]
        # Get from the db
        rows = self.cursor.execute(
            "SELECT timestamp, data FROM messages WHERE topic_id = {}".format(topic_id)
        ).fetchall()
        # Deserialise all and timestamp them
        if convert_to_csv:
            result = []
            for _, data in rows:
                result.append(
                    message_to_csv(
                        deserialize_message(data, self.topic_msg_message[topic_name])
                    )
                )

            return result
        return [
            (timestamp, deserialize_message(data, self.topic_msg_message[topic_name]))
            for timestamp, data in rows
        ]


def get_topics_info(file):
    """Get topics info from metadata file"""

    # Check for metadata file
    LOG.INFO("Checking for metadata file")
    dirname = os.path.dirname(file)
    metadata_file = os.path.join(dirname, "metadata.yaml")

    if not os.path.exists(metadata_file):
        LOG.CRITICAL("Metafile should be in the same directory as the bag file")
        sys.exit()

    LOG.INFO("Found metadata file: " + metadata_file)
    LOG.INFO("Reading metadata file")
    with open(metadata_file, "r") as f:
        metadata = yaml.load(f, Loader=yaml.FullLoader)["rosbag2_bagfile_information"]
        if metadata["storage_identifier"] != "sqlite3":
            LOG.CRITICAL("Error:Please select a sqlite3 bag file")
            sys.exit()

        topic_info = metadata["topics_with_message_count"]

    topics = [topic_info[i]["topic_metadata"]["name"] for i in range(len(topic_info))]

    return topics


def setup_topic_data(file, topics, app):
    """Setup topic data for conversion"""

    # Add all option and get selected topics
    topics.append("all")
    selected = SimplePyQtGUIKit.GetCheckButtonSelect(
        topics, app=app, msg="Select topics to convert csv files"
    )

    selected_topics = []
    output_dir = os.path.dirname(file)
    output_paths = []
    if selected["all"]:
        selected_topics = topics[:-1]
        for topic in selected_topics:
            output_paths.append(
                os.path.join(output_dir, topic.replace("/", "") + ".csv")
            )
    else:
        for k, v in selected.items():
            if v:
                selected_topics.append(k)
                output_paths.append(
                    os.path.join(output_dir, k.replace("/", "") + ".csv")
                )

    if len(selected_topics) == 0:
        LOG.CRITICAL("Select alteast one topic to convert")
        sys.exit()

    LOG.INFO(f"Selected {len(selected_topics)} topics")
    LOG.INFO(f"Output directory: {output_dir}")

    return (selected_topics, output_paths)


def main():
    app = QtWidgets.QApplication(sys.argv)

    while True:
        try:
            # GetFilePath
            files = SimplePyQtGUIKit.GetFilePath(
                isApp=True, caption="Select bag file", filefilter="*db3"
            )
            files = files[:-1]

            if len(files) < 1:
                LOG.CRITICAL("Please select a bag file")
                sys.exit()

            LOG.INFO(f"Selected {len(files)} bag files")

            for file in files:
                topics = get_topics_info(file)
                (selected_topics, output_paths) = setup_topic_data(file, topics, app)
                parser = BagFileParser(file)

                for topic, path in zip(selected_topics, output_paths):
                    LOG.INFO(f"Converting {topic} to {path}")
                    messages = parser.get_messages(topic, convert_to_csv=True)
                    with open(path, "w") as f:
                        f.writelines(messages)

            QtWidgets.QMessageBox.information(
                QtWidgets.QWidget(), "Message", "Finish Convert!!"
            )

        except KeyboardInterrupt:
            LOG.INFO("Ctrl+C pressed. Exiting...")
            sys.exit()
        except Exception as e:
            LOG.CRITICAL(e)
            sys.exit()


if __name__ == "__main__":
    main()
