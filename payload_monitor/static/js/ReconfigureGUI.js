class ReconfigureInput
{
    constructor(inputDescription) {
        this.inputDescription = inputDescription;
        this.name = inputDescription.name;
        this.onchange_callback = undefined;
        this.dom = document.createElement("p");
        //this.dom = document.createElement("li");
    }

    set_value(value) {
        console.log("Value change in undefined field type", this.name, value);
    }

    get_value() {
        return 0;
    }
    
    onchange() {
        if(this.onchange_callback !== undefined)
            this.onchange_callback(this.name, this.get_value());
    }

    set_onchange_callback(callback) {
        // callback signature must be callback(var_name, value);
        this.onchange_callback = callback;
    }
};

class ReconfigureSelector extends ReconfigureInput
{
    constructor(inputDescription) {
        super(inputDescription);
        if(inputDescription.edit_method === "") {
            console.error(inputDescription);
            throw Error("Could not build ReconfigureInutSelector");
        }

        // creating selector from input description
        this.selector = document.createElement("select");
        for(const optDesc of inputDescription.edit_method.enum) {
            let opt = document.createElement("option");
            opt.setAttribute("value", optDesc.value.toString());
            opt.innerHTML = optDesc.name;
            this.selector.appendChild(opt);
        }
        this.selector.onchange = this.onchange.bind(this);

        // building gui
        //let title = document.createElement("span");
        let title = document.createElement("label");
        //let title = document.createElement("a");
        //title.classList.add("subheader"); // prevent the tooltip from working
        title.classList.add("tooltipped");
        title.setAttribute("data-position", "bottom");
        title.setAttribute("data-tooltip", inputDescription.description);
        title.innerHTML = inputDescription.name;

        let div = document.createElement("div");
        div.classList.add("input-field");
        div.appendChild(this.selector);
        div.appendChild(title);
        
        this.dom.appendChild(div);
    }

    set_value(value) {
        this.selector.value = value.toString();
        M.FormSelect.init(this.selector);
    }

    get_value() {
        if(this.inputDescription.type == 'int' || this.inputDescription.type == 'double') {
            return Number(this.selector.value);
        }
        else {
            return this.selector.value;
        }
    }
};

class ReconfigureCheckbox extends ReconfigureInput
{
    constructor(inputDescription) {
        super(inputDescription);
        if(inputDescription.type !== "bool") {
            console.error(inputDescription);
            throw Error("Could not build ReconfigureCheckbox");
        }
        // creating checkbox for input selection
        this.checkbox = document.createElement("input");
        this.checkbox.setAttribute("type", "checkbox");
        this.checkbox.setAttribute("checked", "checked");
        this.checkbox.classList.add("filled-in");
        this.checkbox.onchange = this.onchange.bind(this);

        // building gui
        //let title = document.createElement("a");
        let title = document.createElement("span");
        //title.classList.add("subheader"); // prevent the tooltip from working
        title.classList.add("tooltipped");
        title.setAttribute("data-position", "bottom");
        title.setAttribute("data-tooltip", inputDescription.description);
        title.innerHTML = inputDescription.name;
        
        let label = document.createElement("label");
        label.appendChild(this.checkbox);
        label.appendChild(title);

        this.dom.appendChild(label);
    }

    set_value(value) {
        this.checkbox.checked = value;
    }

    get_value() {
        return this.checkbox.checked;
    }
};
        // parentElement is the parent DOM element this gui will be included in.

class ReconfigureRange extends ReconfigureInput
{
    constructor(inputDescription) {
        super(inputDescription);
        if(!(inputDescription.type === "int" || inputDescription.type === "double")) {
            console.error(inputDescription);
            throw Error("Could not build ReconfigureRange");
        }

        // creating range for input selection
        this.range = document.createElement("input");
        this.range.setAttribute("type", "range");
        this.range.setAttribute("min",  inputDescription.min);
        this.range.setAttribute("max",  inputDescription.max);
        if(inputDescription.type === "double")
            this.range.setAttribute("step", 0.1);
        this.range.onchange = this.onchange.bind(this);

        let title = document.createElement("span");
        //let title = document.createElement("a");
        //title.classList.add("subheader"); // prevent the tooltip from working
        title.classList.add("tooltipped");
        title.setAttribute("data-position", "bottom");
        title.setAttribute("data-tooltip", inputDescription.description);
        title.innerHTML = inputDescription.name;

        this.dom.appendChild(title);
        this.dom.appendChild(this.range);
        this.dom.classList.add("range-field");
    }

    set_value(value) {
        this.range.value = Number(value);
    }

    get_value() {
        return Number(this.range.value);
    }
};


// main reconfigure gui class
class ReconfigureGUI extends ReconfigureClient
{
    constructor(parentElement, target)
    {
        super(target);
        
        this.inputs        = {}; // will contain all classes managing inputs
        this.configRequest = {};

        // parentElement is the parent DOM element this gui will be included in.
        this.parentElement = parentElement;

        // Main gui configuration
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
        // Emptying configRequest to avoid dangling request values not coherent
        // with the GUI.
        this.configRequest = {};
        for(let name in config) {
            if(name in this.inputs) {
                this.inputs[name].set_value(config[name]);
            }
        }
    }

    on_description(configDesc) {
        this.configDescription = configDesc;
        this.generate_gui(this.configDescription);
    }

    generate_gui(configDesc) {
        this.inputs = {}
        while(this.configDiv.firstChild) {
            this.configDiv.removeChild(this.configDiv.firstChild);
        }
        
        let form = document.createElement("form");
        form.setAttribute("action", "#");
        for(const inputDescription of configDesc) {
            let newInput = this.create_input(inputDescription);
            newInput.set_onchange_callback(this.option_changed.bind(this));
            this.inputs[newInput.name] = newInput;
            form.appendChild(newInput.dom);
        }
        this.configDiv.appendChild(form);

        // (TODO) would be better if specific to each input
        $(".tooltipped").tooltip();
        $("select").formSelect();
        M.Range.init($("[type|=range]"));
    }

    create_input(inputDesc) {
        if(inputDesc.edit_method !== "") {
            return new ReconfigureSelector(inputDesc);
        }
        else if(inputDesc.type === "bool") {
            return new ReconfigureCheckbox(inputDesc);
        }
        else if(inputDesc.type === "int" || inputDesc.type === "double") {
            return new ReconfigureRange(inputDesc);
        }
        else {
            console.log("Unknown input type");
            return new ReconfigureInput(inputDesc);
        }
    }
};

