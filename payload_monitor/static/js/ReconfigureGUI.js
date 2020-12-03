

class ReconfigureGUI extends ReconfigureClient
{
    constructor(parentElement)
    {
        super('oculus_sonar');

        this.parentElement = parentElement;

        this.elements = {};
        this.configRequest = {};
        this.configCallbacks = {};

        // all display will be put inside this
        this.mainDiv       = document.createElement("div");
        this.configDiv     = document.createElement("div");
        this.requestButton = document.createElement("a");
        this.requestButton.classList.add("btn", "waves-effect", "waves-light");
        this.requestButton.innerHTML = "Update Configuration";
        this.requestButton.onclick = this.request_config.bind(this);

        this.mainDiv.appendChild(this.configDiv);
        this.mainDiv.appendChild(this.requestButton);

        this.parentElement.appendChild(this.mainDiv);
    }

    option_changed(optionName, value) {
        this.configRequest[optionName] = value;
        console.log(this.configRequest);
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
        for(let name in config) {
            if(name in this.configCallbacks) {
                this.configCallbacks[name](config[name]);
            }
        }
    }

    on_description(configDesc) {
        console.log("Got description");
        this.configDescription = configDesc;

        this.generate_gui(this.configDescription);
    }

    generate_gui(configDesc) {
        this.elements = {}
        while(this.configDiv.firstChild) {
            this.configDiv.removeChild(this.configDiv.firstChild);
        }
        
        let form = document.createElement("form");
        form.setAttribute("action", "#");
        for(const inputDescription of configDesc) {
            let newInput = this.create_input(inputDescription);

            this.elements[newInput.name] = newInput;
            form.appendChild(newInput.dom);
        }
        this.configDiv.appendChild(form);

        $(".tooltipped").tooltip();
        $("select").formSelect();
        M.Range.init($("[type|=range]"));
    }

    create_input(inputDesc) {
        let res = {name        : inputDesc["name"],
                   description : inputDesc};

        if(inputDesc.edit_method !== "") {
            res.dom = this.create_selector(inputDesc);
        }
        else if(inputDesc.type === "bool") {
            res.dom = this.create_checkbox(inputDesc);
        }
        else if(inputDesc.type === "int" || inputDesc.type === "double") {
            res.dom = this.create_range(inputDesc);
        }
        else {
            console.log("Unknown input type");
            res.dom = undefined;
        }

        return res;
    }

    create_selector(inputDesc) {
        
        //let title = document.createElement("span");
        let title = document.createElement("label");
        //let title = document.createElement("a");
        //title.classList.add("subheader"); // prevent the tooltip from working
        title.classList.add("tooltipped");
        title.setAttribute("data-position", "bottom");
        title.setAttribute("data-tooltip", inputDesc.description);
        title.innerHTML = inputDesc.name;

        let div = document.createElement("div");
        div.classList.add("input-field");

        let select = document.createElement("select");
        for(const optDesc of inputDesc.edit_method.enum) {
            let opt = document.createElement("option");
            opt.setAttribute("value", optDesc.value.toString());
            opt.innerHTML = optDesc.name;
            select.appendChild(opt);
        }
        div.appendChild(select);
        div.appendChild(title);
        
        select.target = this;
        select.optionName = inputDesc.name;
        select.onchange = function() {
            this.target.option_changed(this.optionName, Number(this.value));
        }

        function update(value) {
            this.value = value.toString();
            M.FormSelect.init(this);
        }
        update = update.bind(select);
        this.configCallbacks[inputDesc.name] = update;

        let dom   = document.createElement("p");
        //let dom   = document.createElement("li");
        //dom.appendChild(title);
        dom.appendChild(div);
        return dom;
    }

    create_checkbox(inputDesc) {
        let dom   = document.createElement("p");
        //let dom   = document.createElement("li");

        //let title = document.createElement("a");
        let title = document.createElement("span");
        //title.classList.add("subheader"); // prevent the tooltip from working
        title.classList.add("tooltipped");
        title.setAttribute("data-position", "bottom");
        title.setAttribute("data-tooltip", inputDesc.description);
        title.innerHTML = inputDesc.name;
        
        let label = document.createElement("label");

        let checkBox = document.createElement("input");
        checkBox.setAttribute("type", "checkbox");
        checkBox.setAttribute("checked", "checked");
        checkBox.classList.add("filled-in");

        checkBox.target = this;
        checkBox.optionName = inputDesc.name;
        checkBox.onchange = function() {
            this.target.option_changed(this.optionName, this.checked);
        }

        function update(value) {
            this.checked = value;
        }
        update = update.bind(checkBox);
        this.configCallbacks[inputDesc.name] = update;

        label.appendChild(checkBox);
        label.appendChild(title);

        dom.appendChild(label);
        
        return dom;
    }

    create_range(inputDesc) {
        let dom   = document.createElement("p");
        //let dom   = document.createElement("li");

        let title = document.createElement("span");
        //let title = document.createElement("a");
        //title.classList.add("subheader"); // prevent the tooltip from working
        title.classList.add("tooltipped");
        title.setAttribute("data-position", "bottom");
        title.setAttribute("data-tooltip", inputDesc.description);
        title.innerHTML = inputDesc.name;

        let input = document.createElement("input");
        input.setAttribute("type", "range");
        input.setAttribute("min",  inputDesc.min);
        input.setAttribute("max",  inputDesc.max);
        if(inputDesc.type === "double")
            input.setAttribute("step", 0.1);

        input.target = this;
        input.optionName = inputDesc.name;
        input.onchange = function() {
            this.target.option_changed(this.optionName, Number(this.value));
        }

        function update(value) {
            this.value = value;
        }
        update = update.bind(input);
        this.configCallbacks[inputDesc.name] = update;

        dom.appendChild(title);
        dom.appendChild(input);
        dom.classList.add("range-field");

        return dom;
    }
};

