

// main reconfigure gui class
class RecorderGUI extends ReconfigureClient
{
    constructor(reconfContainer)
    {
        super('/ws/recorder_client/');

        this.isRecording = false;
        
        this.container = reconfContainer;
    }

    request_config() {
        if(jQuery.isEmptyObject(this.configRequest)) {
            console.log("Empty request");
            return;
        }
        this.websocket.send(JSON.stringify(
            {type    : "config_request",
             payload : this.configRequest}));
    }
    
    on_config(config) {
        if(config.recording) {
            this.statusDisplay.innerHTML = "Recording in progress";
            this.stopStartBtn.innerHTML  = "Stop Recording";
            this.confirmationMessage.innerHTML = "Confirm stop recording ?";
            this.isRecording = true;
        }
        else {
            this.statusDisplay.innerHTML = "Recording not active";
            this.stopStartBtn.innerHTML  = "Start Recording";
            this.confirmationMessage.innerHTML = "Confirm start recording ?";
            this.isRecording = false;
        }
    }

    on_description(configDesc) {
        this.configDescription = configDesc;
        console.log("Got description here");
        console.log(configDesc);
        this.generate_gui(this.configDescription);
    }
    
    generate_gui(configDesc) {
        // Clear current configuration
        while(this.container.firstChild) {
            this.container.removeChild(this.container.firstChild);
        }

        // // Main gui configuration
        // let title   = document.createElement("a");
        // title.innerHTML = "Recorder Status :";
        // let titleLi = document.createElement("li");
        // titleLi.appendChild(title);
        // this.container.appendChild(titleLi);

        this.statusDisplay = document.createElement("a");
        this.stopStartBtn  = document.createElement("a");
        //this.stopStartBtn.onclick = this.toggle_recording.bind(this);
        this.stopStartBtn.classList.add("waves-effect", "waves-light", "btn");

        for(let param of configDesc) {
            if(param.name === "recording") {
                if(param.current_value) {
                    this.statusDisplay.innerHTML = "Recording in progress";
                    this.stopStartBtn.innerHTML  = "Stop Recording";
                    this.isRecording = true;
                }
                else {
                    this.statusDisplay.innerHTML = "Recording not active";
                    this.stopStartBtn.innerHTML  = "Start Recording";
                    this.isRecording = false;
                }
            }
        }
        let statusLi = document.createElement("li");
        statusLi.appendChild(this.statusDisplay);
        this.container.appendChild(statusLi);

        let btnLi = document.createElement("li");
        btnLi.appendChild(this.stopStartBtn);
        this.container.appendChild(btnLi);
        
        this.generate_confirmation_gui();
    }

    generate_confirmation_gui() {
        let modalDiv = document.createElement("div");
        modalDiv.classList.add("modal");
        modalDiv.id = "confirmation-modal";
        
        // Modal content 
        let modalContent = document.createElement("div");
        modalContent.classList.add("modal-content");
        this.confirmationMessage = document.createElement("h4");
        if(this.isRecording)
            this.confirmationMessage.innerHTML = "Confirm stop recording ?";
        else
            this.confirmationMessage.innerHTML = "Confirm start recording ?";
        this.confirmationMessage.style.color = "black";
        this.confirmationMessage.style.textAlign = "center";
        modalContent.appendChild(this.confirmationMessage);
        modalDiv.appendChild(modalContent);
        
        // Modal footer (buttons)
        let modalFooter = document.createElement("div");
        modalFooter.classList.add("modal-footer");

        let confirmBtn = document.createElement("a");
        confirmBtn.classList.add("modal-close", "waves-effect", "waves-green", "btn-flat");
        confirmBtn.innerHTML = "Confirm"
        confirmBtn.onclick = this.toggle_recording.bind(this);
        modalFooter.appendChild(confirmBtn);

        let cancelBtn = document.createElement("a");
        cancelBtn.classList.add("modal-close", "waves-effect", "waves-green", "btn-flat");
        cancelBtn.innerHTML = "Cancel"
        modalFooter.appendChild(cancelBtn);

        modalDiv.appendChild(modalFooter);

        this.stopStartBtn.href = "#confirmation-modal";
        this.stopStartBtn.classList.add("modal-trigger");

        this.container.appendChild(modalDiv);

        $('.modal').modal();

        //console.log(this.container);
    }

    toggle_recording() {
        if(this.isRecording) {
            this.websocket.send(JSON.stringify(
                {type    : "config_request",
                 payload : { recording : false } }));
        }
        else {
            this.websocket.send(JSON.stringify(
                {type    : "config_request",
                 payload : { recording : true } } ));
        }
    }
};

