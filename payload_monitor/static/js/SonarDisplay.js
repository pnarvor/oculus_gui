class SonarLayout
{
    constructor(mainContainer)
    {
        this.mainContainer = mainContainer;

        // Creating a grid layout to hold the display area and the buttons.
        this.grid = document.createElement("div")
        this.grid.classList.add("row");
        
        // Left column holding the control buttons.
        this.btnColumn = document.createElement("div");
        this.btnColumn.classList.add("col");
        this.btnColumn.classList.add("s1");
        
        // vertical flip button
        this.vFlipButton = SonarLayout.create_button();
        this.vFlipButton.innerHTML = "Flip V";
        let row = document.createElement("div");
        row.classList.add("row");
        row.appendChild(this.vFlipButton);
        this.btnColumn.appendChild(row);

        this.hFlipButton = SonarLayout.create_button();
        this.hFlipButton.innerHTML = "Flip H";
        row = document.createElement("div");
        row.classList.add("row");
        row.appendChild(this.hFlipButton);
        this.btnColumn.appendChild(row);
        
        // Right column holding the display
        this.displayColumn = document.createElement("div");
        this.displayColumn.classList.add("col");
        this.displayColumn.classList.add("s11");

        this.grid.appendChild(this.btnColumn);
        this.grid.appendChild(this.displayColumn);

        this.mainContainer.appendChild(this.grid);
    }

    static create_button() {
        let button = document.createElement("a");
        button.classList.add("waves-effect");
        button.classList.add("waves-light");
        button.classList.add("btn-large");

        return button;
    }
};

class SonarDisplay extends Display
{
    constructor(mainContainer, identifier = "sonar_display")
    {
        let layout = new SonarLayout(mainContainer);

        // Creating a new canvas before using base class constructor.
        let canvas = document.createElement("canvas");
        canvas.setAttribute("id", identifier);
        layout.displayColumn.appendChild(canvas);

        super(canvas);

        this.pingRenderer = new SonarRenderer(this.gl);
        this.add_renderer(this.pingRenderer);

        this.pingListener = new RosTopicListener('/ping', 'oculus_sonar/OculusPing');
        this.pingListener.callbacks.push(this.ping_callback.bind(this));

        this.busy = false;

        this.layout = layout;
        this.layout.vFlipButton.onclick = 
            this.pingRenderer.vertical_flip.bind(this.pingRenderer);
        this.layout.hFlipButton.onclick = 
            this.pingRenderer.horizontal_flip.bind(this.pingRenderer);
    }

    async ping_callback(content)
    {
        // does this "mutex" really is secure ? (this function is
        // asynchronously called by the callback of a websocket.
        if(this.busy) {
            //console.log("Busy : ignoring data");
            return;
        }
        this.busy = true;
        
        try {
            let metadata = JSON.parse(content.content.scalars);
            let data = new Uint8Array(await content.fetch_cached_data('data'));

            this.pingRenderer.set_ping_data(metadata, data.subarray(metadata.imageOffset));
        }
        finally {
            this.busy = false;
        }
        this.draw();
    }
};

