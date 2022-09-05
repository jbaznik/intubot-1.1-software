//show or hide expert mode
let expertModeOn = false;
let automode_available = false;

const triggerExpertMode = () => {
  if (!expertModeOn) {
    document.getElementById('expertmode').style.width = '300px';
    document.getElementById('video').style.width = 'calc(100% - 300px)';
    document.getElementById('borderbox').style.width = 'calc(100% - 300px)';
  } else {
    document.getElementById('expertmode').style.width = '0%';
    document.getElementById('video').style.width = '100%';
    document.getElementById('borderbox').style.width = '100%';
  }
  expertModeOn = !expertModeOn;
};

//show messages received from backend
const popupMessage = message => {
  let msgBox = document.getElementById('messagebox');
  msgBox.innerHTML = message;
  msgBox.style.opacity = 1;
  setTimeout(() => {
    msgBox.style.opacity = 0;
  }, 1500);
};

//change logo color
const changeLogo = automode_available_new => {
  if (automode_available != automode_available_new) {
    let logo = document.getElementById('logo');
    if (automode_available_new) {
      logo.src = 'static/logo-green.png';
    } else {
      logo.src = 'static/logo-white.png';
    }
    automode_available = automode_available_new;
    logo.style.width = '200px';
    logo.style.height = '200px';
    setTimeout(() => {
      logo.style.width = '100px';
      logo.style.height = '100px';
    }, 300);
  }
};

//draw border
const drawBorder = velocities => {
  const element = document.getElementById('borderbox');
  let color;
  let offset_x;
  let offset_y;
  let blur_radius;
  let spread_radius;
  const opacity = velocities['z_dot'] / Math.sqrt(0.5);
  if (automode_available) {
    color = 'rgba(153, 254, 176, ' + opacity + ')';
  } else {
    color = 'rgba(255, 255, 255, ' + opacity + ')';
  }
  if (velocities['x_dot'] > 0) {
    offset_x = '20px';
  } else if (velocities['x_dot'] < 0) {
    offset_x = '-20px';
  } else {
    offset_x = '0px';
  }
  if (velocities['y_dot'] > 0) {
    offset_y = '20px';
  } else if (velocities['y_dot'] < 0) {
    offset_y = '-20px';
  } else {
    offset_y = '0px';
  }
  if (velocities['x_dot'] == 0 && velocities['y_dot'] == 0 && velocities['z_dot'] > 0) {
    spread_radius = '20px';
  } else {
    spread_radius = '0px';
  }
  if (velocities['x_dot'] == 0 && velocities['y_dot'] == 0 && velocities['z_dot'] == 0) {
    blur_radius = '0px';
  } else {
    blur_radius = '20px';
  }
  element.style['box-shadow'] =
    offset_x + ' ' + offset_y + '  ' + blur_radius + ' ' + spread_radius + ' ' + color + ' inset';
};

const drawDeflection = velocities => {
  //draw deflection graphics
  const canvas = document.getElementById('deflection');
  const context = canvas.getContext('2d');
  context.clearRect(0, 0, canvas.width, canvas.height);
  context.beginPath();
  context.arc(100, 100, 95, 0, 2 * Math.PI);
  context.lineWidth = 2;
  context.strokeStyle = '#2370A6';
  context.stroke();
  context.beginPath();
  context.moveTo(100, 100);
  context.lineTo(100 - 100 * velocities['x_dot'], 100 - 100 * velocities['y_dot']);
  context.stroke();
};

//data stream
const sse = new EventSource('/data_feed');
sse.onmessage = e => {
  const data = JSON.parse(e.data);
  changeLogo(data.automode_available);
  drawBorder(data.velocities);
  drawDeflection(data.velocities);
  // write technical data into expertmode window
  let text = '<h2>Network</h2>';
  for (let entry of data.network) {
    for (let [key, value] of Object.entries(entry)) {
      text += key + ': ' + value + '<br />';
    }
  }
  text += '<h2>Motors</h2>';
  for (let [key, value] of Object.entries(data.motors)) {
    text += key + ': ' + value + '<br />';
  }
  text += '<h2>Auto Mode</h2>';
  text += 'available: ' + data.automode_available + '<br />';
  text += 'active: ' + data.automode + '<br />';
  text += '<h2>Detections</h2>';
  for (let entry of data.detections) {
    for (let [key, value] of Object.entries(entry)) {
      text += key + ': ' + value + '<br />';
    }
  }
  const element = document.getElementById('databox');
  element.innerHTML = text;
};

sse.onerror = e => {
  console.error('error data feed');
  sse.close();
};

//handle keyboard events
window.onkeydown = e => {
  if (!e.repeat) {
    //fire only once when key is getting pressed
    if (['i', 'o'].includes(e.key)) {
      let req = new XMLHttpRequest();
      req.open('POST', '/key_down', true);
      req.setRequestHeader('content-type', 'application/json;charset=UTF-8');
      req.send(JSON.stringify({key: e.key}));
    } else {
      console.log(e.key);
      //no command found for that key
    }
  }
};

window.onkeyup = e => {
  if (e.key == ' ') {
    triggerExpertMode();
  } else if (['c', 'h', 'i', 'm', 'o', 'Escape'].includes(e.key)) {
    let req = new XMLHttpRequest();
    req.onreadystatechange = function () {
      if (this.readyState == 4 && this.status == 200) {
        const res = JSON.parse(this.responseText);
        if (res.message) {
          popupMessage(res.message);
        }
      }
    };
    req.open('POST', '/key_up', true);
    req.setRequestHeader('content-type', 'application/json;charset=UTF-8');
    req.send(JSON.stringify({key: e.key}));
  } else {
    console.log(e.key);
    //no command found for that key
  }
};
