const div_console = document.getElementById("div-console")
const img_eyes = document.getElementById("img-eyes")
const p_text = document.getElementById("p-text");


const COMMAND_URL = "http://elmo:8000/api/onboard/command";
const STATE_URL = "http://elmo:8000/api/onboard/state";


const show = (element) => {
    element.style.display = "block";
}


const hide = (element) => {
    element.style.display = "none";
}


hide(div_console);


const log = (msg) => {
    div_console.textContent = msg.toUpperCase();
    console.log(msg);
}


const loadImage = (image_name) => {
    log("loading image: " + image_name)
    img_eyes.src = image_name;
}

const setText = (text) => {
    log("set text: " + text);
    p_text.textContent = text.toUpperCase();
}


const ONBOARD_STATE = {
    "image": "",
    "text": "",
}


async function loop() {
    // get command
    const result = await fetch(COMMAND_URL);
    const data = await result.json();
    // process command
    if (data.image) {
        show(img_eyes);
    } else {
        hide(img_eyes);
    }
    if (data.image !== ONBOARD_STATE.image) {
        loadImage(data.image);
        ONBOARD_STATE.image = data.image;
    }
    if (data.text) {
        show(p_text);
    } else {
        hide(p_text);
    }
    if (data.text !== ONBOARD_STATE.text) {
        setText(data.text);
        ONBOARD_STATE.text = data.text;
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
    postJSON(STATE_URL, ONBOARD_STATE)
}


setInterval(loop, 100);
