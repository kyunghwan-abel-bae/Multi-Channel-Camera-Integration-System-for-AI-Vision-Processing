/*

Color
* basic - #484848
* lighter - #6e6e6e
* darker - #2a2a2a
* highlight - #f05a28

Image
* icon - https://www.iconfinder.com/search/?q=server&iconset=picol-vector
* icon(refresh) - https://www.iconfinder.com/editor/?id=126579&hash=c0c1a3dae82d737e7f9ff6cc672b28f6361fa347091f2bf451e7e870
* icon(radio button - not clicked) - https://www.iconfinder.com/editor/?id=326676&hash=db9dd5fe836eb8ed22cc6b03d91bb0ccb7c8152958eba0d3629c8e34
* icon(radio button - clicked) - https://www.iconfinder.com/editor/?id=326677&hash=29e9f3d8d0019446759fe28f0f7f5a676da9172387f2f219a7ffae06
* icon(bandwidth) - https://www.iconfinder.com/editor/?id=7549065&hash=20dc15fdf7026a2f8e897e6e918bec10821383794db9fc1ab16d1a3e
* icon(bandwidth) - https://www.iconfinder.com/editor/?id=7549065&hash=20dc15fdf7026a2f8e897e6e918bec10821383794db9fc1ab16d1a3e
* icon(exposure) - https://www.iconfinder.com/icons/352333/exposure_icon
* icon(frame) - https://www.iconfinder.com/editor/?id=211745&hash=1f034b9f963998eb2c013c3d91030eae1b05ca56694cfbb783265f0d
* icon(LED) - https://www.iconfinder.com/editor/?id=4752993&hash=b17e6f7df35e31723bd403f18a4d84d0ee0f8ef233500166e564227e

*/

import QtQuick 2.13
import QtQuick.Controls 2.12
import QtQuick.Window 2.13

import BackendManager 1.0

import "qml/parts"

Rectangle {
    id: root

    width: 1544
    height: 872

    visible: true

    color: "#2a2a2a"

    // constant properties for states
    readonly property int k_align_server_connecting: 0
    readonly property int k_align_server_disconnected: 1
    readonly property int k_align_server_connected: 2
    readonly property int k_align_data_received: 3
    readonly property int k_align_data_sent: 4
    readonly property int k_align_analyzing: 5

    readonly property int k_grip_server_listening: 0
    readonly property int k_grip_server_closed: 1
    readonly property int k_grip_client_connected: 2
    readonly property int k_grip_data_received: 3
    readonly property int k_grip_data_sent: 4
    readonly property int k_grip_analyzing: 5

    // flag properties for quit processes
    property bool qmlReadyToQuit: false
    property bool triggerQuit: false

    // flag property to show that it is currently recording.
    property bool isRecordingOn

    // states for devices
    property int alignState: 0
    property int gripState: 0

    // current coil thickness
    property int currentCoilThickness: -1

    // system logs
    property string strSystemLog: ""
    property string strSystemLogCurrentTime: ""

    // index-based device info group
    property var groupDeviceInfoName: []
    property var groupDeviceInfoID: []

    Component.onCompleted: {
        alignState = k_align_server_disconnected
        gripState = k_grip_server_closed
    }

    onTriggerQuitChanged: {
        safe_destroy()
    }

    onAlignStateChanged: {
        var m_align_state = alignState

        switch(m_align_state) {
        case k_align_server_connecting:
            set_front_log(qsTr("Connecting to OPC server..."))
            break
        case k_align_server_connected:
            set_front_log(qsTr("OPC server connected"))
            break
        case k_align_server_disconnected:
            set_front_log(qsTr("OPC server disconnected"))
            break
        case k_align_data_received:
            set_front_log(qsTr("Data Received"))
            var current_coil_thickness = backendManager.q_get_current_coil_thickness() * 1
            var current_rebar_shape = backendManager.q_get_current_rebar_shape()

            if(current_coil_thickness === 10) {
                partTopMenu.aliasTopMenuRebarAlignment.aliasRadioCoil10.activate_button_clicked()
                set_front_log(qsTr("10mm coil data received"))
            }else if(current_coil_thickness === 13){
                partTopMenu.aliasTopMenuRebarAlignment.aliasRadioCoil13.activate_button_clicked()
                set_front_log(qsTr("13mm coil data received"))
            }else if(current_coil_thickness === 16) {
                partTopMenu.aliasTopMenuRebarAlignment.aliasRadioCoil16.activate_button_clicked()
                set_front_log(qsTr("16mm coil data received"))
            }

            partTopMenu.aliasTopMenuRebarAlignment.aliasTextRebarShape.text = current_rebar_shape

            timerAlignState.change_state_to(k_align_analyzing, 2000)
            break
        case k_align_analyzing:
            break
        case k_align_data_sent:
            timerAlignState.change_state_to(k_align_analyzing, 2000)
            break
        }

        alignState = m_align_state
    }

    onGripStateChanged: {
        var m_grip_state = gripState

        switch(m_grip_state) {
        case k_grip_server_listening:
            set_front_log(qsTr("Robot server listening to the robot client..."))
            break
        case k_grip_client_connected:
            set_front_log(qsTr("Robot client connected"))
            break
        case k_grip_server_closed:
            set_front_log(qsTr("Server closed"))
            break
        case k_grip_data_received:
            set_front_log(qsTr("Grip point data received"))
            break
        case k_grip_analyzing:
            set_front_log(qsTr("Vision point arrived"))
            backendManager.grip_get_coordinates()
            break
        case k_grip_data_sent:
            set_front_log(qsTr("Data sent to the robot client"))
            timerGripState.change_state_to(k_grip_client_connected, 2000)
            break
        }

        gripState = m_grip_state
    }

    // Some quit processes take time and resources,
    // so SW is going to check status every second till 10 secs
    function safe_destroy() {
        console.log("safe destroy")
        var count = backendManager.count_running_devices()
        var count_uploaded_files = backendManager.count_uploaded_files()

        if(count === 0 && count_uploaded_files === 0)
            timerSafeDestroy.timeDeadline = -1

        if(count > 0 && !timerSafeDestroy.running)
            backendManager.stop_all_stream()

        if(count_uploaded_files > 0 && !timerSafeDestroy.running)
            backendManager.remove_uploaded_files()

        if(!timerSafeDestroy.running)
            timerSafeDestroy.start()
    }

    // show log in the ui
    function set_front_log(log) {
        var d = new Date()
        var date = d.getFullYear() + "-" + (d.getMonth()+1) + "-" + d.getDate()
        var time = d.getHours() + ":" + d.getMinutes() + ":" + d.getSeconds()

        if(strSystemLog.length > 0) {
            strSystemLog = log + "\n" + strSystemLogCurrentTime + strSystemLog

        } else
            strSystemLog = log + "\n" + strSystemLog

        strSystemLogCurrentTime = "[" + date + " " + time + "] "
    }

    // backendmanager in python
    BackendManager {
        id: backendManager

        // the result of robot-vision calibration
        property double gripDeltaX: 0
        property double gripDeltaY: 0

        // for generated test data
        property double gripRobotDeltaX: 0
        property double gripRobotDeltaY: 0

        property double fpsAligningView
        property double fpsShapingView

        property double exposureAligningView
        property double exposureShapingView

        signal runServer()
        signal stopServer()
        signal runRobotServer()
        signal stopRobotServer()

        signal runDevice(var device_name)
        signal stopDevice(var device_name)

        signal startVisionAt(var device_name)
        signal stopVisionAt(var device_name)

        signal startRecordAt(var device_name)
        signal stopRecordAt(var device_name)

        signal turnOnLEDAt(var device_name)
        signal turnOffLEDAt(var device_name)

        Component.onCompleted: {
            streamingDone.connect(remove_output)

            init()
        }

        onGripDeltaUpdated: {
            var list = get_grip_delta_values()

            gripDeltaX = list[0]
            gripDeltaY = list[1]
        }

        onGripRobotDeltaUpdated: {
            var list = get_grip_robot_delta_values()
            gripRobotDeltaX = list[0]
            gripRobotDeltaY = list[1]
        }

        onBackendAlignStateChanged: {
            if(backendAlignState == "ServerConnected")
                alignState = k_align_server_connected
            else if(backendAlignState == "ServerConnecting")
                alignState = k_align_server_connecting
            else if(backendAlignState == "DataReceived")
                alignState = k_align_data_received
            else if(backendAlignState == "ServerDisconnected")
                alignState = k_align_server_disconnected
            else if(backendAlignState == "DataSent")
                alignState = k_align_data_sent
        }

        onBackendRobotStateChanged: {
            if(backendRobotState == "ClientConnected")
                gripState = k_grip_client_connected
            else if(backendRobotState == "ServerListening")
                gripState = k_grip_server_listening
            else if(backendRobotState == "DataReceived")
                gripState = k_grip_data_received
            else if(backendRobotState == "Analyzing")
                gripState = k_grip_analyzing
            else if(backendRobotState == "ServerClosed")
                gripState = k_grip_server_closed
            else if(backendRobotState == "DataSent")
                gripState = k_grip_data_sent
        }

        onRunServer: {
            q_run_server()
        }

        onStopServer: {
            q_stop_server()
        }

        onRunRobotServer: {
            q_run_robot_server()
        }

        onStopRobotServer: {
            q_stop_robot_server()
        }

        onTurnOnLEDAt: {
            q_turn_on_led_at(device_name)
        }

        onTurnOffLEDAt: {
            q_turn_off_led_at(device_name)
        }

        onLogChanged: {
            set_front_log(log)
        }

        function init() {
            q_init()
        }

        // save ini file to gsa2.ini
        function save_ini() {
            q_save_ini()
        }

        // 1. run device using device_name and device id
        // 2. make space for video stream in ui
        // When you want to use this function outside of DeviceItem,
        // use request_run_stream_by_name(device_name) in DeviceList.qml
        //            partLeftBody.aliasDeviceList.request_stop_stream_by_name(qsTr("Rebar Twist"))
        function run_stream(device_name, device_id) {
            qmlReadyToQuit = false

            console.log("device_name : " + device_name)

            var ret = -1
            var device_output = partRightBody.create_output(device_name, device_id)

            if(device_output === null)
                return ret

            set_front_log(qsTr("Run streaming : " + device_name))
            ret = q_run_device(device_output, device_name, device_id)

            if(ret === -1) {
                partRightBody.remove_output(device_id)
            }

            return ret
        }

        function stop_stream(device_id) {
            set_front_log(qsTr("Stop streaming : " + device_id))
            q_stop_device(device_id)
        }

        // stop aligning view
        // aligining = aligning, tuning, manual tuning
        function stop_aligning_view() {
            q_save_tuning_view_value()

            var index = groupDeviceInfoName.indexOf("Aligning View")
            if(index === -1) {
                dialogOK.title = qsTr("   Device ID for Aligning View not registered..   ")
                dialogOK.open()
                return
            }

            var id_align_view = groupDeviceInfoID[index]
            if(id_align_view === "") {
                dialogOK.title = qsTr("   Device ID for Aligning View not registered..   ")
                dialogOK.open()
                return
            }

            stop_stream(id_align_view)
        }

        function start_vision_at(device_name) {
            q_start_vision_by_name(device_name)
        }

        function stop_vision_at(device_name) {
            q_stop_vision_by_name(device_name)
        }

        // shaping = shaping, manual grip tuining
        function stop_shaping_view() {
            console.log("stop shape view")

            var index = groupDeviceInfoName.indexOf("Shaping View")
            if(index === -1) {
                dialogOK.title = qsTr("   Device ID for Shaping View not registered..   ")
                dialogOK.open()
                return
            }

            var id_shape_view = groupDeviceInfoID[index]
            if(id_shape_view === "") {
                dialogOK.title = qsTr("   Device ID for Shaping View not registered..   ")
                dialogOK.open()
                return
            }

            stop_stream(id_shape_view)
        }

        function stop_all_stream() {
            q_stop_all_devices()
        }

        function remove_output(device_id) {
            partRightBody.remove_output(device_id)
        }

        function send_data() {
            q_send_data()
            return true
        }

        function count_running_devices() {
            return q_count_running_device_objs()
        }

        function count_uploaded_files() {
            return q_count_uploaded_files()
        }

        function remove_uploaded_files() {
            return q_remove_uploaded_files()
        }

        function start_manual_grip_tuning() {
            q_start_manual_grip_tuning()
        }

        function end_manual_grip_tuning() {
            q_end_manual_grip_tuning()
        }

        function generate_test_data_for_manual_grip_tuning(delta_x, delta_y) {
            q_generate_test_data_for_manual_grip_tuning(delta_x, delta_y)
        }

        function save_grip_delta_value(delta_x, delta_y) {
            q_save_grip_delta_value(delta_x, delta_y)
        }

        // This function is used at the backend
        function copy_saved_device_info_qml2py() {
            q_copy_saved_device_info_qml2py(groupDeviceInfoID, groupDeviceInfoName)

            partLeftBody.aliasDeviceList.request_update_device_name()
        }

        // This function is used at the backend
        function copy_saved_device_info_py2qml(list_device_id, list_device_name) {
            groupDeviceInfoID = list_device_id
            groupDeviceInfoName = list_device_name
        }

        function copy_saved_fps_expo_py2qml(list_aligning_view_info, list_shaping_view_info) {
            fpsAligningView = list_aligning_view_info[0]
            exposureAligningView = list_aligning_view_info[1]

            fpsShapingView = list_shaping_view_info[0]
            exposureShapingView = list_shaping_view_info[1]
        }

        // this is deprecated, it is existed for test
        function grip_get_background() {
            q_grip_get_background()
        }

        // this is deprecated, it is existed for test
        function grip_get_coordinates() {
            q_grip_get_coordinates()
        }

        function get_opc_server_state() {
            return q_get_opc_server_state()
        }

        function get_scanned_device_ids() {
            return q_get_scanned_device_ids()
        }

        function get_tuning_values() {
            return q_get_tuning_values()
        }

        function get_grip_delta_values() {
            return q_get_grip_delta_values()
        }

        function get_grip_robot_delta_values() {
            return q_get_grip_robot_delta_values()
        }

        function get_led_status_by_name(name) {
            return q_get_led_status_by_name(name)
        }

        function get_alignment_value() {
            q_get_alignment_value()
        }

        function get_opc_server_option() {
            return q_get_opc_server_option()
        }

        function get_robot_server_option() {
            return q_get_robot_server_option()
        }

        function update_robot_server_option(ip, port, auto_connection) {
            q_update_robot_server_option(ip, port, auto_connection)
        }

        function update_opc_server_option(ip, port, auto_connection) {
            q_update_opc_server_option(ip, port, auto_connection)
        }

        function update_record_on_at(device_id, value) {
            q_update_record_on_at(device_id, value)
        }

        function update_led_status_at(device_id, value) {
            q_update_led_status_at(device_id, value)
        }

        function update_led_status_by_name(device_name, value) {
            q_update_led_status_by_name(device_name, value)
        }

        function update_fps_at(device_id, value) {
            q_update_fps_at(device_id, value)
        }

        function update_fps_by_name(device_name, value) {
            q_update_fps_by_name(device_name, value)
        }

        function update_exposure_at(device_id, value) {
            q_update_exposure_at(device_id, value)
        }

        function update_exposure_by_name(device_name, value) {
            q_update_exposure_by_name(device_name, value)
        }

        function update_current_coil_thickness(value) {
            q_update_current_coil_thickness(value)
        }

        function update_start_in_cell_visible(value) {
            q_update_start_in_cell_visible(value)
        }

        function update_start_in_cell_size(value) {
            q_update_start_in_cell_size(value)
        }

        function update_start_in_cell_x(value) {
            q_update_start_in_cell_x(value)
        }

        function update_start_in_cell_y(value) {
            q_update_start_in_cell_y(value)
        }

        function update_start_out_cell_visible(value) {
            q_update_start_out_cell_visible(value)
        }

        function update_start_out_cell_size(value) {
            q_update_start_out_cell_size(value)
        }

        function update_start_out_cell_x(value) {
            q_update_start_out_cell_x(value)
        }

        function update_start_out_cell_y(value) {
            q_update_start_out_cell_y(value)
        }

        function update_end_in_cell_visible(value) {
            q_update_end_in_cell_visible(value)
        }

        function update_end_in_cell_size(value) {
            q_update_end_in_cell_size(value)
        }

        function update_end_in_cell_x(value) {
            q_update_end_in_cell_x(value)
        }

        function update_end_in_cell_y(value) {
            q_update_end_in_cell_y(value)
        }

        function update_end_out_cell_visible(value) {
            q_update_end_out_cell_visible(value)
        }

        function update_end_out_cell_size(value) {
            q_update_end_out_cell_size(value)
        }

        function update_end_out_cell_x(value) {
            q_update_end_out_cell_x(value)
        }

        function update_end_out_cell_y(value) {
            q_update_end_out_cell_y(value)
        }

        function update_manual_current_out_cell_x(value) {
            q_update_manual_current_out_cell_x(value)
        }

        function update_manual_current_out_cell_y(value) {
            q_update_manual_current_out_cell_y(value)
        }

        function update_manual_current_in_cell_x(value) {
            q_update_manual_current_in_cell_x(value)
        }

        function update_manual_current_in_cell_y(value) {
            q_update_manual_current_in_cell_y(value)
        }

        function update_alignment_interval(value) {
            q_update_alignment_interval(value)
        }
    }

    QtObject {
        id: frontendManager

        function check_polar_coords_visible(checked) {
            if(checked) {
                partRightBody.set_polar_coords_visible_at(0, true)
            }
            else {
                partRightBody.set_polar_coords_visible_at(0, false)
            }
        }

        function update_polar_coords_opacity(value) {
            partRightBody.set_polar_coords_opacity_at(0, value)
        }
    }

    // Timer for safe destroy
    Timer {
        id: timerSafeDestroy
        running: false; interval: 1000; repeat: true
        property int timeElapsed: 0
        property int timeDeadline: 10000

        onTriggered: {
            safe_destroy()

            timeElapsed += interval

            set_front_log(qsTr("Closing SW in ") + ((10000-timeElapsed)/1000) + " seconds..")

            if(timeElapsed > timeDeadline) {
                timerSafeDestroy.stop()
                qmlReadyToQuit = true

                backendManager.save_ini()
                Qt.quit()
            }
        }
    }

    // Timer for align state changes
    Timer {
        id: timerAlignState
        running: false; interval: 500; repeat: true

        property int timeElapsed: 0
        property int timeEnd: 0
        property int toState: -1

        onTriggered: {
            timeElapsed += interval
            if(timeElapsed >= timeEnd) {
                alignState = toState

                running = false
                timeElapsed = 0
                timeEnd = 0
                toState = -1
            }
        }

        function change_state_to(arg_state, time) {
            toState = arg_state
            timeEnd = time
            timerAlignState.running = true
        }
    }

    // Timer for grip state changes
    Timer {
        id: timerGripState
        running: false; interval: 500; repeat: true
        property int timeElapsed: 0
        property int timeEnd: 0
        property int toState: -1

        onTriggered: {
            timeElapsed += interval
            if(timeElapsed >= timeEnd) {
                gripState = toState

                running = false
                timeElapsed = 0
                timeEnd = 0
                toState = -1
            }
        }

        function change_state_to(arg_state, time) {
            toState = arg_state
            timeEnd = time
            timerGripState.running = true
        }
    }

    Dialog {
        id: dialogOK
        standardButtons: Dialog.Ok

        anchors.centerIn: parent
    }

    PartTopMenu {
        id: partTopMenu

        width: parent.width; height: parent.height * 0.13
    }

    Item {
        id: itemBody

        width: parent.width; height: parent.height - partTopMenu.height - 2
        anchors.bottom: parent.bottom

        PartLeftBody {
            id: partLeftBody

            width: parent.width * 0.15; height: parent.height
        }

        PartRightBody {
            id: partRightBody
            width: parent.width - partLeftBody.width; height: parent.height
            anchors.right: parent.right
        }
    }
}
