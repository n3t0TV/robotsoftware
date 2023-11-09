const puppeteer = require('puppeteer');
//const rosnodejs = require('rosnodejs');
//rosnodejs.loadAllPackages();
//const StringMessage = rosnodejs.require('std_msgs').msg.String;
//console.log('StringMessage',rosnodejs.require('std_msgs').msg.String)
//const config = require('../config.js');


var url = "https://example.com";
//var pubControl=null;

class HeadlessBrowser {

    constructor() {
       // this.launchBrowser();

    }

    async launchBrowser() {
        this.browser = await puppeteer.launch({
            chromeFlags: [
                '--disable-gpu',
                '--headless',
                '‑‑netifs‑to‑ignore=wifi0'
            ],
            args: ['--no-sandbox', '--disable-setuid-sandbox']
        });
    }

    async goto(path) {
        const page = await this.browser.newPage();
        const fullpath = path;
        console.log(`test start at ${fullpath}`);

        setInterval(async () =>{
            console.log("Snapshot");
            page.screenshot({path: 'testsnap.png'});
            /*const result = await page.evaluate(()=>{

                return;
            });*/
            //sendControlMessage(result);
            //console.log('controlJSon', result);
         } ,3000);
        const pageResult = await page.goto(fullpath);

        return page;
    }

}
/*
function sendControlMessage(controlJson)
{
    //console.log(pubControl)
    if(pubControl!=null && controlJson!=null && controlJson!= 'undefined')
    {
        //console.log('control',controlJson);
        //var str = JSON.stringify(controlJson);
        //console.log(str);
        pubControl.publish(new StringMessage({data:controlJson}));
    }
}
*/


async function  openBrowser()
{
    var browser = new HeadlessBrowser();
    await browser.launchBrowser();
    console.log("browsaer created!");

     page = await browser.goto(url);
    console.log("browser launched!");

}

openBrowser();
