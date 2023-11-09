//const express = require("express");
//const exec = require("child_process").exec;
const puppeteer = require('puppeteer')
const rosnodejs = require('rosnodejs');
const { init } = require("rosnodejs/dist/utils/network_utils");
// rosnodejs.loadAllPackages();

//const port = 8080;
//exec("http-bridge -C --remote-host 34.127.27.210 --forward-port "+port);


class HeadlessBrowser {

    constructor() {
       // this.launchBrowser();
    }

    async launchBrowser() {
        this.browser = await puppeteer.launch({
            chromeFlags: [
                '--disable-gpu',
                '--headless'
            ],
            args: ['--no-sandbox', '--disable-setuid-sandbox']
        });
    }

    async goto(path) {
        const page = await this.browser.newPage();
        const fullpath = path;
        console.log(`test start at ${fullpath}`);

        /*setInterval(async () =>{
            console.log("Snapshot");
            page.screenshot({path: 'testsnap.png'});

            //sendControlMessage(result);
            //console.log('controlJSon', result);
         } ,3000);*/
        const pageResult = await page.goto(fullpath);

        return page;
    }

}


class DashboardConnection{
    constructor(url){
        //const app = express();
        //app.use(express.static(__dirname + "/public"));
        /*this.httpServer = require("http").createServer(app);
        this.httpServer.listen(port);
        this.initSocket();*/
        var dashboadConnection = this;
        this.openBrowser(url);
        setInterval(async () =>{
            const result = await dashboardConnection.page.evaluate(()=>{

                return window.messageReceived;
            });
            dashboardConnection.receivedMessage(result);
            //console.log('controlJSon', result);
         } ,1000);

    }

    async openBrowser(url)
    {
        var browser = new HeadlessBrowser();
        await browser.launchBrowser();
        console.log("browser created!");

        var dashboardConnection = this;
        this.page = await browser.goto(url);
        console.log("browser launched!");
    }

    setRos(dashboardRos){
        this.dashboardRos = dashboardRos;
    }

  /*  async initSocket(){
        this.socketIo = require('socket.io')(this.httpServer,{ cors:{ origin:"*"},allowEIO3:true});
        //this.socketIo.on("connection",this.onConnection.bind(this));
    }*/

    /*onConnection(socket){
        console.log("Connection successfull!");
      //  socket.join("dashboardcito");
    }*/
    receivedMessage(result)
    {
      console.log("Pulling!!",result);
    }

    async sendMessage(sioevent,msg){
      await this.page.evaluate(() => {
        sendMessage ('Hello from brayan!');
      });
        //this.socketIo.to("dashboardcito").emit(sioevent,msg);
    }
}

class DashboardRos{
    constructor(dashboardConnection){
        this.dashboardConnection = dashboardConnection;
        this.initializeNode();
        this.dashboardConnection.setRos(this);
    }

    initializeNode()
    {
        rosnodejs.initNode('/dashboard_node').then((nh) =>{
            console.log('Dashboard node initialized!');

            //nh.subscribe('diagnostic_topic/gps', 'drivers/gps_dgnst_msg', this.gpsCallback.bind(this));
            //nh.subscribe('exception_topic', 'std_msgs/Int32', this.exceptionCallback.bind(this));
        })
    }

    gpsCallback(msg){
        // console.log("<-- gps");
        this.dashboardConnection.sendMessage('gps',msg);
    }

    exceptionCallback(msg){
        // console.log("<-- exception");
        this.dashboardConnection.sendMessage('exception',msg);
    }
}

const dashboardConnection = new DashboardConnection("http:///192.168.1.66/html/ProxyVehicle.html?imei=7891&proxySession=1234");
const dashboardRos = new DashboardRos(dashboardConnection);
