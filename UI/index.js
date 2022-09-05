document.getElementById("myButton").onclick = function() {
    var myName = document.getElementById("myText").value;
    document.getElementById("demo").innerHTML = myName;
    console.log("hello", myName)
  }