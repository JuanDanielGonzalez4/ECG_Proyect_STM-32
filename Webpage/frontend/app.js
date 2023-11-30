/**
 * Add gobals here
 */
var yValues = [];
var xValues = [];
var array_length = 100;
var counter = 0;
var socket;

const data = {
  labels: xValues,
  datasets: [
    {
      label: 'ECG',
      data: [],
      backgroundColor: "#1de9b6",
      borderColor: "#1de9b6 ",
    },
  ]
};

const config = {
  type: 'line',
    data,
    options: {
      plugins: {
        legend: {
          position: 'top',
        },
        title: {
          display: true,
          text: 'Electrocardiogram'
        }
      },
      // scales: {
      //   x: {
      //     realtime: {
      //       onRefresh: chart => {
      //         chart.data.datasets.forEach(dataset => {
      //           dataset.data.push({
      //             x: xValues,
      //             y: yValues,
      //           });
      //         });
      //       }
      //     }
      //   }
      // },
      responsive: true,
      maintainAspectRatio: false
    },
}


function sendAge(){
  var age = document.getElementById("age").value;
  console.log(age)
  socket.send(age);

}

/**
 * Initialize functions here.
 */

document.addEventListener('DOMContentLoaded', (event) => {

  myChart = new Chart(
    document.getElementById('myChart'),
    config
  );

  const outputDiv = document.getElementById('output');

  socket = new WebSocket('ws://localhost:5000');

  socket.onopen = function() {
    console.log("Connection established!");
    socket.send('request_data');
  };

  socket.onmessage = function(event) {
    console.log('Data received from server:', event.data);
    if (event.data){
      outputDiv.innerHTML = 'Received data: ' + event.data;
      console.log(event.data)
      const numericData = parseInt(event.data, 10);
      yValues.push(numericData);
      xValues.push(counter);
  
      if (yValues.length > array_length) {
        yValues.shift(); 
        xValues.shift(); 
      }
  
      myChart.data.datasets[0].data = xValues.map((x, i) => ({ x, y: yValues[i] }));
      myChart.update();

      console.log(data);
  
      counter++;
    }
  };

});


