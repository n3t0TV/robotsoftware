const express = require("express");
const exec = require("child_process").exec;
const rosnodejs = require('rosnodejs');
const { init } = require("rosnodejs/dist/utils/network_utils");
// rosnodejs.loadAllPackages();

const port = 8080;
exec("http-bridge -C --remote-host 34.127.27.210 --forward-port "+port);

class DashboardConnection{
    constructor(){
        const app = express();
        app.use(express.static(__dirname + "/public"));
        this.httpServer = require("http").createServer(app);
        this.httpServer.listen(port);
        this.initSocket();
    }

    setRos(dashboardRos){
        this.dashboardRos = dashboardRos;
    }

    async initSocket(){
        this.socketIo = require('socket.io')(this.httpServer,{ cors:{ origin:"*"},allowEIO3:true});
        this.socketIo.on("connection",this.onConnection.bind(this));
    }

    onConnection(socket){
        console.log("Connection successfull!");
        socket.join("dashboardcito");

        /* para ejecutar una funcion de ros (publicar en un getTopLevelMessageDirectory, pedir un servicio, etc.)
        socket.on('accion1',this.onAccion1.bind(this)); */
    }

    /* onAccion1(msg){
        // funcion de dashboard ros
        this.dashboardRos.funcion
    } */

    async sendMessage(sioevent,msg){
        this.socketIo.to("dashboardcito").emit(sioevent,msg);
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
            
            nh.subscribe('diagnostic_topic/gps', 'drivers/gps_dgnst_msg', this.gpsCallback.bind(this));
            nh.subscribe('exception_topic', 'std_msgs/Int32', this.exceptionCallback.bind(this));
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

const dashboardConnection = new DashboardConnection();
const dashboardRos = new DashboardRos(dashboardConnection);