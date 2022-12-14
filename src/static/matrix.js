let chinese = '田由甲申甴电甶男甸甹町画甼甽甾甿畀畁畂畃畄畅畆畇畈畉畊畋界畍畎畏畐畑';
chinese = chinese.split('');
const font_size = 10;
let drops = [];
let c = document.getElementById('matrix');
let ctx = c.getContext('2d');

//making the canvas full screen
c.height = window.innerHeight;
c.width = window.innerWidth;

const columns = c.width / font_size;

for (var x = 0; x < columns; x++) {
  drops[x] = 1;
}

//drawing the characters
const draw = () => {
  //Black BG for the canvas
  //translucent BG to show trail
  ctx.fillStyle = 'rgba(0, 0, 0, 0.05)';
  ctx.fillRect(0, 0, c.width, c.height);

  ctx.fillStyle = '#0F0'; //green text
  ctx.font = font_size + 'px arial';
  //looping over drops
  for (var i = 0; i < drops.length; i++) {
    //a random chinese character to print
    var text = chinese[Math.floor(Math.random() * chinese.length)];
    //x = i*font_size, y = value of drops[i]*font_size
    ctx.fillText(text, i * font_size, drops[i] * font_size);

    //sending the drop back to the top randomly after it has crossed the screen
    //adding a randomness to the reset to make the drops scattered on the Y axis
    if (drops[i] * font_size > c.height && Math.random() > 0.975) drops[i] = 0;

    //incrementing Y coordinate
    drops[i]++;
  }
};

setInterval(draw, 33);
