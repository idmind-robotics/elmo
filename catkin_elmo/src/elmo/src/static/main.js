const div_console = document.getElementById("div-console");
const img_eyes = document.getElementById("img-eyes");
const p_text = document.getElementById("p-text");
const video = document.getElementById("video");
const menu = document.getElementById("menu");

const menu_call = document.getElementById("menu-call");
const menu_music = document.getElementById("menu-music");
const menu_alarm = document.getElementById("menu-alarm");


const COMMAND_URL = "http://elmo:8000/api/onboard/command";
const STATE_URL = "http://elmo:8000/api/onboard/state";
const USER_REQUEST_URL = "http://elmo:8000/api/onboard/user_request";


const ONBOARD_STATE = {
    "image": null,
    "text": null,
    "video": null,
};


let SHOWING_MENU = false;

const publish_user_request = (r) => {
    // post "call" to user request
    fetch(USER_REQUEST_URL, {
        method: "POST",
        mode: "cors",
        cache: "no-cache",
        headers: {
            "Content-Type": "application/json"
        },
        body: JSON.stringify({"request": r})
    });
    hide(menu);
    SHOWING_MENU = false;
}


menu_call.onclick = () => {
    publish_user_request("call");
};


menu_music.onclick = () => {
    publish_user_request("music");
};


menu_alarm.onclick = () => {
    publish_user_request("alarm");
};


img_eyes.onclick = () => {
    hide(img_eyes);
    show_flex(menu);
    SHOWING_MENU = true;
    window.setTimeout(() => {
        log("reset menu")
        hide(menu);
        SHOWING_MENU = false;
    }, 5000);
};


const reset_state = () => {
    ONBOARD_STATE.image = null;
    ONBOARD_STATE.text = null;
    ONBOARD_STATE.video = null;
};


const show = (element) => {
    element.style.display = "block";
};


const show_flex = (element) => {
    element.style.display = "flex";
};


const hide = (element) => {
    element.style.display = "none";
};


hide(div_console);
hide(menu);
hide(img_eyes);
hide(p_text);
hide(video);

const log = (msg) => {
    div_console.textContent = msg.toUpperCase();
    console.log(msg);
};

const loadImage = (image_name) => {
    log("loading image: " + image_name);
    hide(video);
    hide(p_text);
    img_eyes.src = image_name;
    show(img_eyes);
};

const setText = (text) => {
    log("set text: " + text);
    hide(img_eyes);
    hide(video);
    p_text.textContent = text.toUpperCase();
    show(p_text);
};


START_TIME = 1.0;
END_TIME = 1.2;


const playVideo = ({video_name, start_time, end_time}) => {
    log("play video: " + video_name + " " + start_time + " " + end_time);
    hide(img_eyes);
    hide(p_text);
    hide(video);
    video.addEventListener("loadeddata", () => {
        show(video);
    });
    video.src = video_name;
    video.play();
    // reset state after video ends
    video.addEventListener("ended", () => {
        ONBOARD_STATE.video = null;
    });
};


let last_command = null;
async function loop() {
    if (SHOWING_MENU) {
        return;
    }

    // get command
    const result = await fetch(COMMAND_URL);
    const data = await result.json();
    if (data.command !== last_command) {
        console.log(data);
    }
    last_command = data.command;
    // process command
    if (data.image !== ONBOARD_STATE.image) {
        if (data.image) {
            ONBOARD_STATE.image = data.image;
            loadImage(data.image);
        } else {
            ONBOARD_STATE.image = null;
            hide(img_eyes);
        }
    }
    if (data.text !== ONBOARD_STATE.text) {
        if (data.text) {
            ONBOARD_STATE.text = data.text;
            setText(data.text);
        } else {
            ONBOARD_STATE.text = null;
            hide(p_text);
        }
    }
    if (data.video !== ONBOARD_STATE.video) {
        if (data.video) {
            ONBOARD_STATE.video = data.video;
            playVideo(data.video);
        }
    }
    if (!ONBOARD_STATE.image && !ONBOARD_STATE.text && !ONBOARD_STATE.video) {
        ONBOARD_STATE.image = data.image;
        loadImage("images/normal.png");
    }
    // update state
    await fetch(STATE_URL, { 
        method: "POST",
        mode: "cors",
        cache: "no-cache",
        headers: {
            "Content-Type": "application/json" 
        },
        body: JSON.stringify(ONBOARD_STATE)
    });
};


setInterval(loop, 100);
