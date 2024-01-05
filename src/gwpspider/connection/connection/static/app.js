// static/script.js
console.log("Hello from script.js!");

// Function to start working
function startWorking() {
    // Send an HTTP request to the FastAPI backend
    fetch('/start_working', {
        method: 'POST',
        headers: {
            'Content-Type': 'application/json',
        },
        body: JSON.stringify({}),
    })
    .then(response => {
        if (!response.ok) {
            throw new Error('Network response was not ok');
        }
        return response.json();
    })
    .then(data => {
        console.log('Response from server:', data);
    })
    .catch(error => {
        console.error('There was a problem with the fetch operation:', error);
    });
}

// Function to stop working
function stopWorking() {
    // Send an HTTP request to the FastAPI backend to stop working
    fetch('/stop_working', {
        method: 'POST',
        headers: {
            'Content-Type': 'application/json',
        },
        body: JSON.stringify({}),
    })
    .then(response => {
        if (!response.ok) {
            throw new Error('Network response was not ok');
        }
        return response.json();
    })
    .then(data => {
        console.log('Response from server:', data);
    })
    .catch(error => {
        console.error('There was a problem with the fetch operation:', error);
    });
}

function submitForm() {
    // Get form values
    const x1 = document.getElementById("x1").value;
    const y1 = document.getElementById("y1").value;
    const z1 = document.getElementById("z1").value;
    const coordSystem = document.querySelector('input[name="coord-system"]:checked').value;
    const offset = document.querySelector('input[name="offset"]:checked').value;

    // Create an object with the form data
    const formData = {
        x1: x1,
        y1: y1,
        z1: z1,
        coordSystem: coordSystem,
        offset: offset
    };

    // Send an HTTP request to the FastAPI backend
    fetch('/process_walk_to_position', {
        method: 'POST',
        headers: {
            'Content-Type': 'application/json',
        },
        body: JSON.stringify(formData),
    })
    .then(response => {
        if (!response.ok) {
            throw new Error('Network response was not ok');
        }
        return response.json();
    })
    .then(data => {
        console.log('Response from server:', data);
    })
    .catch(error => {
        console.error('There was a problem with the fetch operation:', error);
    });
}

function submitMoveBodyForm() {
    console.log('Submit button clicked!');
    // Get form values
    const x2 = document.getElementById("x2").value;
    const y2 = document.getElementById("y2").value;
    const z2 = document.getElementById("z2").value;
    const coordSystem2 = document.querySelector('input[name="coord-system2"]:checked').value;
    const offset2 = document.querySelector('input[name="offset2"]:checked').value;

    // Create an object with the form data
    const formData = {
        x2: x2,
        y2: y2,
        z2: z2,
        coordSystem2: coordSystem2,
        offset2: offset2
    };

    // Send an HTTP request to the FastAPI backend
    fetch('/process_move_body', {
        method: 'POST',
        headers: {
            'Content-Type': 'application/json',
        },
        body: JSON.stringify(formData),
    })
    .then(response => {
        if (!response.ok) {
            throw new Error('Network response was not ok');
        }
        return response.json();
    })
    .then(data => {
        console.log('Response from server:', data);
    })
    .catch(error => {
        console.error('There was a problem with the fetch operation:', error);
    });
}

function submitGripperAction(action) {
    // Get the selected radio button value
    const selectedGripper = document.querySelector('input[name="Grippers"]:checked');
    
    if (selectedGripper) {
        const gripperIdx = selectedGripper.value;

        // Create an object with the form data
        const formData = {
            gripperIdx: gripperIdx,
            gripperAction: action
        };

        // Send an HTTP request to the FastAPI backend
        fetch('/process_gripper_action', {
            method: 'POST',
            headers: {
                'Content-Type': 'application/json',
            },
            body: JSON.stringify(formData),
        })
        .then(response => {
            if (!response.ok) {
                throw new Error('Network response was not ok');
            }
            return response.json();
        })
        .then(data => {
            console.log('Response from server:', data);
        })
        .catch(error => {
            console.error('There was a problem with the fetch operation:', error);
        });
    } else {
        console.error('No gripper selected');
    }
}

function submitMoveLegForm() {
    // Get the form elements
    const legRadio = document.querySelector('input[name="Legs"]:checked');
    const x4 = document.getElementById("x4").value;
    const y4 = document.getElementById("y4").value;
    const z4 = document.getElementById("z4").value;
    const coordSystem4 = document.querySelector('input[name="coord-system4"]:checked');
    const offset4 = document.querySelector('input[name="offset4"]:checked');

    if (legRadio && coordSystem4 && offset4) {
        const legIdx = legRadio.value;
        const coordSystemValue = coordSystem4.value;
        const offsetValue = offset4.value;

        // Create an object with the form data
        const formData = {
            legIdx: legIdx,
            x4: x4,
            y4: y4,
            z4: z4,
            coordSystem4: coordSystemValue,
            offset4: offsetValue
        };

        // Send an HTTP request to the FastAPI backend
        fetch('/process_move_leg', {
            method: 'POST',
            headers: {
                'Content-Type': 'application/json',
            },
            body: JSON.stringify(formData),
        })
        .then(response => {
            if (!response.ok) {
                throw new Error('Network response was not ok');
            }
            return response.json();
        })
        .then(data => {
            console.log('Response from server:', data);
        })
        .catch(error => {
            console.error('There was a problem with the fetch operation:', error);
        });
    } else {
        console.error('Incomplete form data');
    }
}

function GoRefill() {
    // Set the refill variable to 1
    const refill = 1;

    // Create an object with the form data
    const formData = {
        refill: refill
    };

    // Send an HTTP request to the FastAPI backend
    fetch('/process_go_refill', {
        method: 'POST',
        headers: {
            'Content-Type': 'application/json',
        },
        body: JSON.stringify(formData),
    })
    .then(response => {
        if (!response.ok) {
            throw new Error('Network response was not ok');
        }
        return response.json();
    })
    .then(data => {
        console.log('Response from server:', data);
    })
    .catch(error => {
        console.error('There was a problem with the fetch operation:', error);
    });
}

function submitPumpAction(action) {
    // Get the selected pump value
    const selectedPump = document.querySelector('input[name="Pumps"]:checked');

    if (selectedPump) {
        const pumpIdx = selectedPump.value;

        // Create an object with the form data
        const formData = {
            pumpIdx: pumpIdx,
            pumpAction: action
        };

        // Send an HTTP request to the FastAPI backend
        fetch('/process_pump_action', {
            method: 'POST',
            headers: {
                'Content-Type': 'application/json',
            },
            body: JSON.stringify(formData),
        })
        .then(response => {
            if (!response.ok) {
                throw new Error('Network response was not ok');
            }
            return response.json();
        })
        .then(data => {
            console.log('Response from server:', data);
        })
        .catch(error => {
            console.error('There was a problem with the fetch operation:', error);
        });
    } else {
        console.error('No pump selected');
    }
}


// Function to handle the "Activate" button click
document.getElementById("breaksAct").addEventListener("click", function () {
    fetch('/activate_breaks')  // Make a GET request to activate_breaks endpoint
        .then(response => response.json())
        .then(data => {
            console.log(data.message);
            // You can add additional client-side logic if needed
        })
        .catch(error => {
            console.error('Error:', error);
        });
});

// Function to handle the "Deactivate" button click
document.getElementById("breaksDeact").addEventListener("click", function () {
    fetch('/deactivate_breaks')  // Make a GET request to deactivate_breaks endpoint
        .then(response => response.json())
        .then(data => {
            console.log(data.message);
            // You can add additional client-side logic if needed
        })
        .catch(error => {
            console.error('Error:', error);
        });
});
