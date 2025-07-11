
function playMostRecent() {
  var webms = ''
  fetch(`https://localhost:8000/webms.txt`)
    .then(response => response.text())
    .then((data) => {
      webms = data.split(/\r?\n/);
      console.log(webms);
      fetch(`http://localhost:4000/s/${webms[webms.length-1]}`, {
          method: "GET",
          headers: {
            "Content-Type": "application/json",
          },
        })
      })
    .catch(() => {
      console.log("Couldnt get webms");
    });
};