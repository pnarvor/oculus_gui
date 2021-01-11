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
        if(inputDescription.edit_method.type !== "enum") {
            console.error(inputDescription);
            throw Error("Could not build ReconfigureInutSelector");
        }

        // creating selector from input description
        this.selector = document.createElement("select");
        for(const optDesc of inputDescription.edit_method.entries) {
            let opt = document.createElement("option");
            opt.setAttribute("value", optDesc.value.toString());
            opt.innerHTML = optDesc.name;
            this.selector.appendChild(opt);
        }
        this.selector.value = inputDescription.current_value.toString();
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
        if(inputDescription.edit_method.type !== "bool") {
            console.error(inputDescription);
            throw Error("Could not build ReconfigureCheckbox");
        }
        // creating checkbox for input selection
        this.checkbox = document.createElement("input");
        this.checkbox.setAttribute("type", "checkbox");
        this.checkbox.setAttribute("checked", "checked");
        this.checkbox.classList.add("filled-in");
        this.checkbox.checked = inputDescription.current_value;
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

class ReconfigureRange extends ReconfigureInput
{
    constructor(inputDescription) {
        super(inputDescription);
        if(inputDescription.edit_method.type !== "range") {
            console.error(inputDescription);
            throw Error("Could not build ReconfigureRange");
        }

        // creating range for input selection
        this.range = document.createElement("input");
        this.range.setAttribute("type", "range");
        this.range.setAttribute("min",  inputDescription.edit_method.min);
        this.range.setAttribute("max",  inputDescription.edit_method.max);
        if(inputDescription.type === "double")
            this.range.setAttribute("step", 0.1);
        this.range.value = Number(inputDescription.current_value);
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
    constructor(reconfContainer, target)
    {
        super(target);
        
        this.container = reconfContainer;
        
        this.inputs        = {}; // will contain all classes managing inputs
        this.configRequest = {};

        // Main gui configuration
        this.configDiv     = document.createElement("div");
        this.generate_sidenav();
        this.sidenav.appendChild(this.configDiv);
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
            if(inputDescription.edit_method.type === "fixed")
                continue;
            let newInput = this.create_input(inputDescription);
            newInput.set_onchange_callback(this.option_changed.bind(this));
            this.inputs[newInput.name] = newInput;
            form.appendChild(newInput.dom);
        }
        this.configDiv.appendChild(form);

        this.requestButton = document.createElement("a");
        this.requestButton.classList.add("btn", "waves-effect", "waves-light");
        this.requestButton.innerHTML = "Update Configuration";
        this.requestButton.onclick = this.request_config.bind(this);
        this.configDiv.appendChild(this.requestButton);

        // Adding a divider at the end to add some vertical space. (Otherwise,
        // bottom scroll seems not low enough and hide request button.)
        for(let i = 0; i < 3; i ++) {
            let vspace = document.createElement("br");
            //vspace.classList.add("divider");
            //vspace.height = "100px";
            let p = document.createElement("p");
            p.appendChild(vspace);
            this.configDiv.appendChild(p);
        }


        // (TODO) would be better if specific to each input
        $(".tooltipped").tooltip();
        $("select").formSelect();
        M.Range.init($("[type|=range]"));
    }

    create_input(inputDesc) {
        console.log(inputDesc);
        switch(inputDesc.edit_method.type) {
            case "fixed":
                console.log(inputDesc.name, "Fixed");
                break;
            case "bool":
                return new ReconfigureCheckbox(inputDesc);
                break;
            case "range":
                return new ReconfigureRange(inputDesc);
                break;
            case "unbounded":
                console.log(inputDesc, "Unbounded parameter type not implemented yet")
                return new ReconfigureInput(inputDesc);
                break;
            case "enum":
                return new ReconfigureSelector(inputDesc);
                break;
            default:
                console.log(inputDesc.name, "Unknown type :", inputDesc.edit_method.type);
                return new ReconfigureInput(inputDesc);
                break;
        }
    }

    generate_sidenav() {
        let elms = this.container.getElementsByClassName("reconf-sidenav");
        if(elms.length > 0) {
            this.sidenav = elms[0];
            return 0;
        }

        // creating side nav to contain reconfigure inputs
        this.sidenav = document.createElement("ul");
        this.sidenav.id = "slide-out";
        this.sidenav.classList.add("sidenav", "reconf-sidenav");
        this.container.appendChild(this.sidenav);
        
        let reconfOpenBtn = document.createElement("a");
        reconfOpenBtn.classList.add("sidenav-trigger", "show-on-large");
        reconfOpenBtn.setAttribute("href", "#");
        reconfOpenBtn.setAttribute("data-target", "slide-out");
        let icon = document.createElement("i");
        icon.classList.add("material-icons");
        icon.innerHTML = "settings";
        reconfOpenBtn.appendChild(icon);

        let btnContainer = document.createElement("li");
        btnContainer.appendChild(reconfOpenBtn);
        //btnContainer.innerHTML = "<a href=\"{% url 'status'  %}\">Status</a>"
        $("#nav-big-right")[0].appendChild(btnContainer);

        $('.sidenav').sidenav({edge:'right',
                               onOpenEnd  : window.onresize,
                               onCloseEnd : window.onresize});
    }
};

