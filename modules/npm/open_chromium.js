const puppeteer = require('puppeteer');

var browser;

class HeadlessBrowser {

    constructor(headless) {
       this.headless = headless;
       this.broadcasterPage = null;
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
            console.log("Can't launch browser: ", error);
       });
    }

    async closeBrowser() {
        await this.browser.close();
        this.browser = null;
        this.broadcasterPage = null;
        console.log("Browser closed!!");
    }

    async launchBroadcaster(path, cameraCheck) {
        this.broadcasterPage = await this.goto(path);

        this.broadcasterPage.on('console', (msg) => console.log('Page log: ', msg.text()));

        if (cameraCheck) {
            this.labelTimer = setInterval(async () => {
                const result = await this.broadcasterPage.evaluate((command)=>{
                    const res = eval(command)
                    console.log(res)
                    if (res === undefined || res === null) return String(res)
                    return res
                }, 'deviceLabel').catch((error) => {
                    console.log("Can't evaluate broadcaster: ", error);
                });
                console.log(result)
                if (result == "camera_front") {
                    console.log('Encontre la camara')
                    clearInterval(this.labelTimer)
                } else {
                    await this.broadcasterPage.evaluate((command)=>{
                        eval(command)
                    }, 'nextCamera()').catch((error) => {
                        console.log("Can't evaluate broadcaster: ", error);
                    });
                }
            }, 5000);
        }
    }

    async goto(path) {
        const page = await this.browser.newPage();
        page.setCacheEnabled(false);
        const fullpath = path;
        console.log(`goto start at ${fullpath}`);

        const pageResult = await page.goto(fullpath);

        return page;
    }
}

async function launchChromium() {
    const myArgs = process.argv.slice(2);
    console.log('Starting JS ros node..');
    const isHeadless = true;
    browser = new HeadlessBrowser(isHeadless); //init browser
    const urlBroadcaster = myArgs[0]//'https://dev.teleop.tortops.com/dashboard/groupbroadcast?imei=359514062141384';
    await browser.launchBrowser();
    console.log('Browser launched');
    await browser.launchBroadcaster(urlBroadcaster, true);
    console.log("browser launch Broadcaster url!");
}

if (require.main === module) {
    launchChromium();
}
