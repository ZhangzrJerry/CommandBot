/**
 * Team 8214's 2025 Custom Node Selection Panel, written in HTML and
 * [NT4JS](https://github.com/Mechanical-Advantage/RobotCode2023/blob/main/src/main/deploy/nodeselector/NT4.js)
 *
 * @author [zhangzrjerry](https://github.com/zhangzrjerry)
 */

// Disable right click
document.addEventListener('contextmenu', event => event.preventDefault());

// print("means the status of NT4", 2, "green");
// print("means the basic info of NT4", 2, "black");
// print("means trigger by robot", 1, "purple");
// print("means trigger by dashboard", 3, "blue");
// print("means alerts", 2, "red");
console.log("      ____  __   __  ____    _____   ____  \n     / ___| \\ \\ / / | __ )  | ____| |  _ \\ \n    | |      \\ V /  |  _ \\  |  _|   | |_) |\n    | |___    | |   | |_) | | |___  |  _ < \n     \\____|   |_|   |____/  |_____| |_| \\_\\\n ____  _____ _     _____ ____ _____ ___  ____  \n/ ___|| ____| |   | ____/ ___|_   _/ _ \\|  _ \\ \n\\___ \\|  _| | |   |  _|| |     | || | | | |_) |\n ___) | |___| |___| |__| |___  | || |_| |  _ < \n|____/|_____|_____|_____\\____| |_| \\___/|_| \\_\\\n\n欢迎使用由 NI 电控团队开发的 Cyber Selector 2025. Cyber Selector 2025 是一款用于选择道具放置位置、设置机器人状态、指示剩余时间的自定义操控面板。\n\nZhangzrJerry: https://github.com/zhangzrjerry\n  RockyXRQ  : https://github.com/rockyxrq\n     42     : https://github.com/mirrorcy");

import { NT4_Client } from "./nt4.js";

const MATCH_TIME_TOPIC = "/nodeselector/match_time";
const IS_AUTO_TOPIC = "/nodeselector/is_auto";

let isConnect = false;
let time = 0;
let isAuto = false;

const ROBOT_TO_DASHBOARD_TOPIC = "/nodeselector/node_robot_2_dashboard";
const DASHBOARD_TO_ROBOT_TOPIC = "/nodeselector/node_dashboard_2_robot";

let select = "";
let trough = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0];

const IS_IGNORE_ARM_MOVE_CONDITION_TOPIC = "/nodeselector/is_ignore_arm_move_condition";
const IS_IGNORE_ARM_MOVE_CONDITION_ROBOT_TOPIC = "/nodeselector/is_ignore_arm_move_condition_robot_2_dashboard";
const BOOLEAN_TOPICS = [IS_IGNORE_ARM_MOVE_CONDITION_TOPIC];

let booleanValues = [false];

let client = new NT4_Client(
    window.location.hostname,
    "NodeSelector",
    (topic) => {
    },
    (topic) => {
    },
    (topic, timestamp, value) => {
        if (topic.name === IS_AUTO_TOPIC) {
            isAuto = value;
        } else if (topic.name === MATCH_TIME_TOPIC) {
            time = value;
            renderTime(time, false);
        } else if (topic.name === ROBOT_TO_DASHBOARD_TOPIC) {
            if (isAuto) {
                renderSelected(select);
                triggerSelect(value, false);
            }
        } else if (topic.name === IS_IGNORE_ARM_MOVE_CONDITION_ROBOT_TOPIC) {
            print("Received ignore arm move condition from robot: " + value, 1, "purple");
            document.getElementById('arm-forced').children.namedItem('check').checked = value;
            booleanValues[0] = value;
            // 同步更新到dashboard的topic
            client.addSample(IS_IGNORE_ARM_MOVE_CONDITION_TOPIC, value);
        }
    },
    () => {
        print("Connected to NetworkTables.", 2, "green");
        isConnect = true;
    },
    () => {
        print("Disconnected from NetworkTables.", 2, "red");
        isConnect = false;
        time = renderTime(time);
    }
);

window.addEventListener("load", () => {
    client.subscribe([ROBOT_TO_DASHBOARD_TOPIC, MATCH_TIME_TOPIC, IS_AUTO_TOPIC, IS_IGNORE_ARM_MOVE_CONDITION_ROBOT_TOPIC], false, false, 0.02);
    client.publishTopic(DASHBOARD_TO_ROBOT_TOPIC, "string");
    client.publishTopic(IS_IGNORE_ARM_MOVE_CONDITION_TOPIC, "boolean");
    client.connect();
    triggerBoolean(false, 0);

    // listen to algae click
    document.querySelectorAll('.algae').forEach((algae) => {
        algae.addEventListener('click', () => {
            renderSelected(select);
            triggerSelect(algae.id, false);
        });
    });

    // listen to branch click
    document.querySelectorAll('.branch').forEach((branch) => {
        branch.addEventListener('click', () => {
            renderSelected(select);
            triggerSelect(branch.id, false);
        });
    });

    // listen to trough click
    document.querySelectorAll('.trough').forEach((trough) => {
        trough.addEventListener('click', () => {
            renderSelected(select);
            triggerSelect(trough.id, false);
        });
    });

    // listen to net click
    document.querySelectorAll('.net').forEach((net) => {
        net.addEventListener('click', () => {
            renderSelected(select);
            triggerSelect(net.id, false);
        });
    });

    // listen to toggle click
    document.getElementById('arm-forced').children.namedItem('check').addEventListener('click', () => {
        triggerBoolean(document.getElementById('arm-forced').children.namedItem('check').checked, 0);
    });

    print("Cyber Selector 2025 is ready.", 2, "green");

    let cnt = 0;
    setInterval(
        function () {
            renderTime();

            // blink every 500ms
            renderSelect(select, cnt > 24);
            cnt++;
            if (cnt > 49) {
                cnt = 0;
            }

            // flush every 100ms
            if (isConnect && cnt % 5 === 0) {
                flush();
            }
        }, 20);
    // loop rate
    // 20ms = 50Hz
});

function triggerSelect(strValue, fromAuto) {
    select = strValue;

    if (strValue.length === 3 && strValue[2] === '1') {
        let idx = strValue[1].charCodeAt(0) - 'A'.charCodeAt(0);
        trough[idx] = trough[idx] + 1;
        if (document.getElementById(strValue)) {
            document.getElementById(strValue).innerHTML = trough[idx].toString();
        }
    }

    if (true || !fromAuto) {
        client.addSample(DASHBOARD_TO_ROBOT_TOPIC, strValue);
        print("Selected " + strValue, 3, "blue");
    } else {
        print(strValue + " is selected", 1, "purple");
    }
}

function triggerBoolean(boolValue, index) {
    booleanValues[index] = boolValue;
    client.addSample(BOOLEAN_TOPICS[index], boolValue);
    print(BOOLEAN_TOPICS[index].split('/')[BOOLEAN_TOPICS[index].split('/').length - 1].toUpperCase() + ": " + boolValue, 3, "blue");
}

function renderSelect(strValue, blink) {
    if (document.getElementById(strValue)) {
        let element = document.getElementById(strValue);
        element.classList.remove('unselected');
        element.classList.remove('selected');
        element.classList.remove('selecting-1');
        element.classList.remove('selecting-2');
        if (blink) {
            element.classList.add('selecting-1');
        } else {
            element.classList.add('selecting-2');
        }
    }
}

function renderSelected(strValue) {
    if (document.getElementById(strValue)) {
        let element = document.getElementById(strValue);
        element.classList.remove('unselected');
        element.classList.remove('selected');
        element.classList.remove('type0');
        element.classList.remove('type1');
        element.classList.remove('type2');
        element.classList.remove('selecting-1');
        element.classList.remove('selecting-2');
        if (strValue.length === 3 && strValue[2] === '1') {
            let idx = strValue[1].charCodeAt(0) - 'A'.charCodeAt(0);
            if (trough[idx] === 0) {
                element.classList.add('type0');
            } else if (trough[idx] === 1) {
                element.classList.add('type1');
            } else {
                element.classList.add('type2');
            }
        } else {
            element.classList.add('selected');
        }
    }
}

function renderTime() {
    if (document.getElementById('time')) {
        let element = document.getElementById('time');
        element.classList.remove('teleop-1');
        element.classList.remove('teleop-2');
        element.classList.remove('teleop-3');
        element.classList.remove('teleop-4');
        element.classList.remove('auto');

        // disconnect
        if (!isConnect) {
            if (time % 2 === 0) {
                element.classList.add('teleop-4');
            } else {
                element.classList.add('teleop-3');
            }
            element.innerHTML = "LOST";
            return time + 1;
        }

        let min = Math.floor(time / 60);
        let sec = time % 60;
        element.innerHTML = min.toString() + " : " + (sec < 10 ? "0" + sec : sec).toString();
        if (isAuto) {
            element.classList.add('auto');
        } else if (time === 0) {
            // default when time is gone
            element.classList.add('teleop-1');
        } else if (time <= 20) {
            // blinking for alert
            if (time % 2 === 0) {
                element.classList.add('teleop-4');
            } else {
                element.classList.add('teleop-3');
            }
        } else if (time <= 30) {
            element.classList.add('teleop-2');
        } else {
            element.classList.add('teleop-1');
        }
        return time;
    }
    return -1;
}

function flush(fromManual = false) {
    if (fromManual) {
        print("Flushing data to NetworkTables...", 2, "blue");
    }

    if (select !== "") {
        client.addSample(DASHBOARD_TO_ROBOT_TOPIC, select);
    }
    for (let i = 0; i < booleanValues.length; i++) {
        client.addSample(BOOLEAN_TOPICS[i], booleanValues[i]);
    }
}

function print(str, type, color) {
    if (type === 1) {
        console.log("%c(" + new Date().toLocaleTimeString() + "): " + str, "color: " + color);
    } else if (type === 2) {
        console.log("%c[" + new Date().toLocaleTimeString() + "]: " + str, "color: " + color);
    } else if (type === 3) {
        console.log("%c{" + new Date().toLocaleTimeString() + "}: " + str, "color: " + color);
    } else {
        console.log(str);
    }
}
