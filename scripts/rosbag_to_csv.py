#!/usr/bin/env python3

import sys
from pyqt_gui import pyqt_gui
from PyQt5 import QtGui, QtWidgets
import rosbag
import rospy
from optparse import OptionParser
from datetime import datetime


def message_to_csv(stream, msg, flatten=False):
    try:
        for s in type(msg).__slots__:
            val = msg.__getattribute__(s)
            message_to_csv(stream, val, flatten)
    except BaseException:
        msg_str = str(msg)
        if msg_str.find(",") != -1:
            if flatten:
                msg_str = msg_str.strip("(")
                msg_str = msg_str.strip(")")
                msg_str = msg_str.strip(" ")
            else:
                msg_str = "\"" + msg_str + "\""
        stream.write("," + msg_str)


def message_type_to_csv(stream, msg, parent_content_name=""):
    try:
        for s in type(msg).__slots__:
            val = msg.__getattribute__(s)
            message_type_to_csv(stream, val, ".".join(
                [parent_content_name, s]))
    except BaseException:
        stream.write("," + parent_content_name)


def format_csv_filename(form, topic_name):
    global seq
    if form is None:
        return "rosbag_to_csv.csv"
    ret = form.replace('%t', topic_name.replace('/', '-'))
    ret = ret[1:]
    return ret


def bag_to_csv(options, fname):
    try:
        bag = rosbag.Bag(fname)
        streamdict = dict()
        stime = None
        if options.start_time:
            stime = rospy.Time(options.start_time)
        etime = None
        if options.end_time:
            etime = rospy.Time(options.end_time)
    except Exception as e:
        rospy.logfatal("Failed to Convert Bag File: %s", e)
        exit(1)
    finally:
        print(f"\nSuccessfully Converted Bag File: {fname}")

    try:
        for topic, msg, time in bag.read_messages(topics=options.topic_names,
                                                  start_time=stime,
                                                  end_time=etime):
            if topic in streamdict:
                stream = streamdict[topic]
            else:
                stream = open(
                    format_csv_filename(
                        options.output_file_format,
                        fname[fname.rfind('/'): -4] + topic),
                    'w')
                streamdict[topic] = stream
                if options.header:
                    stream.write("time")
                    message_type_to_csv(stream, msg)
                    stream.write('\n')

            stream.write(
                datetime.fromtimestamp(
                    time.to_time()).strftime('%Y/%m/%d/%H:%M:%S.%f'))
            message_to_csv(stream, msg, flatten=not options.header)
            stream.write('\n')
        [s.close for s in streamdict.values()]
    except Exception as e:
        rospy.logwarn("Failed: %s", e)
    finally:
        bag.close()


def GetTopicList(path):
    bag = rosbag.Bag(path)
    topics = list(bag.get_type_and_topic_info()[1].keys())
    print("Topics:\n", topics)
    print("\n")
    print("Messages:")
    types = []
    for dict_values in list(bag.get_type_and_topic_info()[1].values()):
        print(dict_values[0])
        types.append(dict_values[0])

    results = []
    for to, ty in zip(topics, types):
        results.append(to)

    return results


def main(options):
    app = QtWidgets.QApplication(sys.argv)

    # GetFilePath
    files = pyqt_gui.GetFilePath(
        isApp=True, caption="Select Bag File(s)", filefilter="*bag")
    print("Files:\n", files)
    print("\n")
    if len(files) < 1:
        print("\nError: Please Select a Bag File!")
        sys.exit()
    topics = GetTopicList(files[0])
    selected = pyqt_gui.GetCheckButtonSelect(
        topics, app=app, msg="Select Topic(s)")

    options.topic_names = []
    for k, v in selected.items():
        if v:
            options.topic_names.append(k)

    if len(options.topic_names) == 0:
        print("\nError: Please Select a Topic!")
        sys.exit()    

    options.output_file_format = "%t.csv"

    print("\nConverting...")
    for i in range(0, len(files) - 1):
        bag_to_csv(options, files[i])

    QtWidgets.QMessageBox.information(
        QtWidgets.QWidget(),
        "Success", "Finished Converting!")


if __name__ == '__main__':
    print("ROS Bag to CSV Conversion Started!\n")
    print("Waiting for ROS Master...\n")
    rospy.init_node('rosbag_to_csv', anonymous=True)
    parser = OptionParser(usage="%prog [options] bagfile")
    parser.add_option("-a", "--all-topics", dest="all_topics",
                      action="store_true",
                      help="export all topics", default=False)
    parser.add_option("-t", "--topic-names", dest="topic_names",
                      action="append",
                      help="white list topic names", metavar="TOPIC_NAME")
    parser.add_option("-s", "--start-time", dest="start_time",
                      help="start time of bagfile", type="float")
    parser.add_option("-e", "--end-time", dest="end_time",
                      help="end time of bagfile", type="float")
    parser.add_option("-n", "--no-header", dest="header",
                      action="store_false", default=True,
                      help="no header")
    (options, args) = parser.parse_args()

    main(options)
