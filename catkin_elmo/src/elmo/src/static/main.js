const div_console = document.getElementById("div-console");
const img_eyes = document.getElementById("img-eyes");
const p_text = document.getElementById("p-text");
const video = document.getElementById("video");



// const COMMAND_URL = "http://elmo:8000/api/onboard/command";
// const STATE_URL = "http://elmo:8000/api/onboard/state";

const COMMAND_URL = "http://localhost:8000/api/onboard/command";
const STATE_URL = "http://localhost:8000/api/onboard/state";

const ONBOARD_STATE = {
    "image": null,
    "text": null,
    "video": null,
};

let SHOWING_MENU = false;

img_eyes.onclick = () => {
    if (!SHOWING_MENU) {
        SHOWING_MENU = true;
        window.setTimeout(() => {
            log("reset menu")
            SHOWING_MENU = false;
        }, 1000);
    }
};

const reset_state = () => {
    ONBOARD_STATE.image = null;
    ONBOARD_STATE.text = null;
    ONBOARD_STATE.video = null;
};


const show = (element) => {
    element.style.display = "block";
};


const hide = (element) => {
    element.style.display = "none";
};


hide(div_console);
hide(img_eyes);
hide(p_text);
hide(video);

const log = (msg) => {
    div_console.textContent = msg.toUpperCase();
    console.log(msg);
};


const loadImage = (image_name) => {
    reset_state();
    log("loading image: " + image_name);
    hide(video);
    hide(p_text);
    img_eyes.src = image_name;
    show(img_eyes);
};

const setText = (text) => {
    reset_state();
    log("set text: " + text);
    hide(img_eyes);
    hide(video);
    p_text.textContent = text.toUpperCase();
    show(p_text);
};


START_TIME = 1.0;
END_TIME = 1.2;


const playVideo = ({video_name, start_time, end_time}) => {
    reset_state();
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
    // menu override
    if (SHOWING_MENU) {
        show(div_console);
        return;
    } else {
        hide(div_console);
    }

    // get command
    const result = await fetch(COMMAND_URL);
    const data = await result.json();
    if (data.command !== last_command) {
        console.log(data);
    }
    last_command = data.command;
    // process command
    if (data.image && data.image !== ONBOARD_STATE.image) {
        loadImage(data.image);
        ONBOARD_STATE.image = data.image;
    }
    if (data.text && data.text !== ONBOARD_STATE.text) {
        setText(data.text);
        ONBOARD_STATE.text = data.text;
    }
    if (data.video && data.video !== ONBOARD_STATE.video) {
        playVideo(data.video);
        // playVideo();
        ONBOARD_STATE.video = data.video;
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
