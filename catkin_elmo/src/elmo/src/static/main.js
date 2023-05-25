const div_console = document.getElementById("div-console");
const img_eyes = document.getElementById("img-eyes");
const p_text = document.getElementById("p-text");
const video = document.getElementById("video");
const video_bg = document.getElementById("video-bg");



// const COMMAND_URL = "http://elmo:8000/api/onboard/command";
// const STATE_URL = "http://elmo:8000/api/onboard/state";

const COMMAND_URL = "http://localhost:8000/api/onboard/command";
const STATE_URL = "http://localhost:8000/api/onboard/state";

const ONBOARD_STATE = {
    "image": null,
    "text": null,
    "video": null,
};


const show = (element) => {
    hide(img_eyes);
    hide(p_text);
    hide(video);
    element.style.display = "block";
};


const hide = (element) => {
    element.style.display = "none";
};


hide(div_console);
// hide(img_eyes);
// hide(p_text);
// hide(video);

const log = (msg) => {
    div_console.textContent = msg.toUpperCase();
    console.log(msg);
};


const loadImage = (image_name) => {
    log("loading image: " + image_name);
    img_eyes.src = image_name;
};

const setText = (text) => {
    log("set text: " + text);
    p_text.textContent = text.toUpperCase();
};


START_TIME = 1.0;
END_TIME = 1.2;


const playVideo = (video_name) => {
    // plays video from start_time to end_time
    log("play video: " + video_name);
    hide(video);
    video.addEventListener("loadeddata", () => {
        show(video);
        video_bg.src = video_name;
        video_bg.play();
    });
    video.src = video_name;
    video.play();
    // reset state after video ends
    video.addEventListener("ended", () => {
        ONBOARD_STATE.video = null;
    });
    
};

const setVideo = (video_name) => {
    log("set video: " + video_name);
    hide(video);
    video_bg.src = video_name;
};

let last_command = null;
async function loop() {
    // get command
    const result = await fetch(COMMAND_URL);
    const data = await result.json();
    if (data.command !== last_command) {
        console.log(data);
    }
    last_command = data.command;
    // process command
    if (data.image && data.image !== ONBOARD_STATE.image) {
        show(img_eyes);
        loadImage(data.image);
        ONBOARD_STATE.image = data.image;
    }
    if (data.text && data.text !== ONBOARD_STATE.text) {
        show(p_text);
        setText(data.text);
        ONBOARD_STATE.text = data.text;
    }
    if (data.video && data.video !== ONBOARD_STATE.video) {
        show(video);
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
// playVideo("http://localhost:8000/videos/elmo_eyes_green.mp4")