#!/usr/bin/env node
'use strict';

const puppeteer = require('puppeteer');
const rosnodejs = require('rosnodejs');

const log = rosnodejs.log;

var imei = null;
var joyBroadcastUrl = null;
var pubControl = null;
var motionMessage = null;
var browser;


class HeadlessBrowser {

    constructor(headless) {
       this.headless = headless;
       this.controlTimer = null;
       this.statsTimer = null;
       this.prevJsonControl = null;
       this.broadcasterPage = null;
       this.qrPage = null;
       this.refreshingTimeout = null;
       this.defaultPath = "about:blank";
       this.isBroadcasting = false;
    }

    async launchBrowser() {
        this.browser = await puppeteer.launch({
            executablePath: '/usr/bin/chromium-browser',
            chromeFlags: [
                '--disable-gpu',
                '--headless'
            ],
            args: ['--no-sandbox', '--disable-setuid-sandbox', '--use-fake-ui-for-media-stream'],
            headless: this.headless
        }).catch((error) => {
            log.error("Can't launch browser: ", error);
        });
        this.broadcasterPage = await this.getPage();
        this.qrPage = await this.getPage();
    }

    async closeBrowser() {
        await this.browser.close();
        this.browser = null;
        log.info("Browser closed!!");
    }

    async clearBrowser() {
        clearInterval(this.controlTimer);
        clearInterval(this.statsTimer);
        clearTimeout(this.refreshingTimeout);
        this.prevJsonControl = null;
        this.broadcasterPage.removeAllListeners('console');
        await this.broadcasterPage.goto(this.defaultPath);
        this.isBroadcasting = false;
        await this.qrPage.goto(this.defaultPath);
        log.info("Browser cleared!!");
    }

    async launchBroadcaster(path, refreshTimer) {
        log.debug(`goto start at ${path}`);
        await this.broadcasterPage.goto(path);
        this.isBroadcasting = true;

        this.broadcasterPage.on('console', (msg) => log.debug('Page log: ', msg.text()));
        this.broadcasterPage.on('error', (msg) => log.error('Page log: ', msg.text()));

        if (refreshTimer) {
            this.statsTimer = setInterval(async () =>{
                const result = await this.broadcasterPage.evaluate(()=>{

                    return window.lastStats;
                }).catch((error) => {
                    log.error("Can't evaluate broadcaster: ", error);
                });
                if (result && result[0] && result[0].framesEncodedPerSecond) {
                    clearTimeout(this.refreshingTimeout);
                    this.refreshingTimeout = null;
                } else {
                    if (!this.refreshingTimeout) {
                        this.refreshingTimeout = setTimeout(() => {
                            this.broadcasterPage.reload();
                            this.refreshingTimeout = null;
                            log.info('Page refreshed!!');
                        }, 60000);
                    }
                }
            } ,1000);
        }
    }

    async launchQr(path) {
        log.debug(`goto start at ${path}`);
        await this.qrPage.goto(path);

        this.controlTimer = setInterval(async () =>{
            const result = await this.qrPage.evaluate(()=>{

                return window.controlJson;
            }).catch((error) => {
                log.error("Can't evaluate qr: ", error);
            });
            if (this.prevJsonControl != result) sendControlMessage(result);
            this.prevJsonControl = result;
         } ,50);
    }

    async getPage() {
        const page = await this.browser.newPage();
        page.setCacheEnabled(false);
        return page;
    }
}

async function broadcasterReload(req, res) {
    if (browser.isBroadcasting) {

        setTimeout(async () => {
            await browser.broadcasterPage.reload();
        }, 500);

        res.success = true
        res.message = "Page reload triggered"
    } else {
        res.success = false
        res.message = "Error: Teleoperation hasn't started"
    }
    return true
}

async function browserScreenshot(req, res) {
    if (browser.isBroadcasting) {
        const path = '/home/jetson/Pictures/puppeteer_screenshot.png'

        await browser.broadcasterPage.screenshot({
            path: path,
            fullPage: true
        });

        res.success = true
        res.message = path
    } else {
        res.success = false
        res.message = "Error: Teleoperation hasn't started"
    }
    return true
}

async function browserConsole(req, res) {
    if (browser.isBroadcasting) {
        res.success = true
        const result = await browser.broadcasterPage.evaluate((command)=>{
            const res = eval(command)
            console.log(res)
            if (res === undefined || res === null) return String(res)
            return JSON.stringify(res)
        }, req.command).catch((error) => {
            log.error("Can't evaluate broadcast console: ", error);
            res.success = false
            return 'NA'
        });

        res.message = result
    } else {
        res.success = false
        res.message = "Error: Teleoperation hasn't started"
    }
    return true
}

function sendControlMessage(controlJson) {
    //log.info(controlJson);
    if (pubControl && controlJson) {
        const controlObj = JSON.parse(controlJson);
        const motion = new motionMessage({
            TimeStamp: controlObj.t,
            Speed: controlObj.s,
            AngularRate: controlObj.a,
            Brake: controlObj.b
        });
        pubControl.publish(motion);
    }
}

function createRosNode() {
    log.info('Starting JS ros node..');
    rosnodejs.initNode('/control_qr_node', { onTheFly: true} )
    .then(async (nh) => {
        log.info('Node initialized!');
        rosnodejs.on('shutdown', async () => {
            await browser.closeBrowser();
        });

        await nh.getParam('/control_qr_node/log_level')
        .then((paramValue) => {
            if (paramValue !== 0) log.getLogger().setLevel(paramValue);
        })
        .catch((error) => {
            log.error("Can't get param: /control_qr_node/log_level");
        });

        motionMessage = rosnodejs.require('modules').msg.motion_ctrl_msg;
        const statusMessage = rosnodejs.require('modules').msg.state_msg;
        const imeiMessage = rosnodejs.require('modules').srv.imei_service;
        const imeiClient = nh.serviceClient('/imei_service', imeiMessage);
        var req = new imeiMessage.Request();

        await nh.waitForService('/imei_service', 3000)
        .then(async (available) => {
            if (available) {
                await imeiClient.call(req)
                .then((res) => {
                    imei = res.imei;
                    log.debug('imei: ' + imei);
                });
            } else {
                log.error('Not imei available');
                rosnodejs.shutdown();
                return;
            }
        });

        const isHeadless = await nh.getParam('/control_qr_node/headless')
        .then((paramValue) => {
            return paramValue;
        })
        .catch((error) => {
            log.error("Can't get param: /control_qr_node/headless");
            return true;
        });

        joyBroadcastUrl = await nh.getParam('/control_qr_node/joyBroadcastUrl')
        .then((paramValue) => {
            return paramValue;
        })
        .catch((error) => {
            log.error("Can't get param: /control_qr_node/joyBroadcastUrl");
            return "";
        });

        log.debug('Param: ', isHeadless)
        browser = new HeadlessBrowser(isHeadless); //init browser
        log.info('Browser class created');
        await browser.launchBrowser();
        log.info('Browser launched');

        const statusSub = nh.subscribe('/status_topic',statusMessage, statusCallback);
        pubControl = nh.advertise('/teleops_topic/qr',motionMessage);
        const screenshotSrv = nh.advertiseService('/puppeteer/screenshot','std_srvs/Trigger',browserScreenshot);
        const consoleSrv = nh.advertiseService('/puppeteer/console','modules/console_service',browserConsole);
        const pageReloadSrv = nh.advertiseService('/puppeteer/page_reload','std_srvs/Trigger',broadcasterReload);
    });
}

async function statusCallback(msg) {
    log.debug('Status msg: ', msg);
    if (msg.status == 5) { //TELEOP

        if(msg.source == "MONITOR") {
          const urlBroadcaster = 'https://' + msg.url_servidor + '/dashboard/groupbroadcast?imei=' + imei;

          await browser.launchBroadcaster(urlBroadcaster, false);
          log.info("browser launch Group Broadcaster url!");
        } else {
          const urlBroadcaster = 'https://' + msg.url_servidor + '/broadcast/#/' + imei + '/' + msg.id_sesion_teleop + '/640/480/0';
          const urlQr = 'https://' + msg.url_servidor + '/qrreceiver.html?imei=' + imei + '&idSession=' + msg.id_sesion_teleop + 'qr';

          await browser.launchBroadcaster(urlBroadcaster, true);
          log.info("browser launch Broadcaster url!");

          await browser.launchQr(urlQr);
          log.info("browser launch Qr url!");
        }
    } else if (msg.status == 4) { //VENDING
        if (msg.url_servidor) {
            const urlBroadcasterVending = msg.url_servidor + '#/' + imei;

            await browser.launchBroadcaster(urlBroadcasterVending, false);
            log.info("browser launch Broadcaster vending url!");
        }
    } else if (msg.status == 6) { //JOYSTICK
        var urlBroadcasterjoystick = '';
        if (msg.url_servidor) {
            urlBroadcasterjoystick = 'https://' + msg.url_servidor + '/dashboard/groupbroadcast?imei=' + imei;
        } else if (joyBroadcastUrl) {
            urlBroadcasterjoystick = joyBroadcastUrl + imei;
        }

        if (urlBroadcasterjoystick) {
            await browser.launchBroadcaster(urlBroadcasterjoystick, false);
            log.info("browser launch Broadcaster joystick url!");
        }
    } else if (msg.status == 3) {
        await browser.clearBrowser();
    }
}

if (require.main === module) {
    createRosNode();
}
