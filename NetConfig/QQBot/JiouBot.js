"use strict";
const { createClient, core } = require("oicq");
const fs = require("fs");
const async = require("async");

const account = 2375614961;
const client = createClient(account);

const HELPTEXT =
	"JiouBot v0.0.5 debug\n" +
	"要触发我，请at我或在消息中加入'jiou'关键词\n" +
	"支持好友私聊\n" +
	"操作间隔低于一分钟自动忽略\n" +
	"用户群 631371483\n" +
	"help 显示此文字\n";

//const RESO = require("./reso.json");
//const PLANT = RESO.plant;
//const SHOP = RESO.shop;

client.on("system.login.qrcode", function (e) {
	console.log("扫码完成后请按回车");
	process.stdin.once("data", () => {
		this.login();
	});
});

client
	.on("system.login.slider", function (e) {
		console.log("输入ticket：");
		process.stdin.once("data", (ticket) => this.submitSlider(String(ticket).trim()));
	})
	.login();

client.on("system.online", () => console.log("系统已登入，当前时间" + Date.now()));

client.on("request.friend", (e) => {
	e.approve(true);
});

client.on("request.group.invite", (e) => {
	e.approve(true);
});

let userTiming = new Map();

//文件读写定义----------------------------------------------------------------
let writing = false;

let compUser = (a, b) => {
	return b.Money - a.Money;
};

let writeJson = (toWrite, path_way) => {
	console.log("尝试在 " + path_way + " 中写入数据 " + JSON.stringify(toWrite));

	fs.readFile(path_way, (err, data) => {
		if (err) return console.error("读取文件失败：", err);

		let thisJSONStr = data.toString(); //将二进制数据转为字符串
		let thisJSON = JSON.parse(thisJSONStr); //将字符串转换为 json 对象
		thisJSON.data.push(toWrite); //将传来的对象 push 进数组对象中

		thisJSON.total = thisJSON.data.length; //定义一下总条数，为以后的分页打基础
		thisJSONStr = JSON.stringify(thisJSON);
		fs.writeFile(path_way, thisJSONStr, (err) => {
			if (err) console.error("重写失败：", err);
			else console.log("写入成功");
		});
	});
};

let deleteJson = (id, path_way) => {
	console.log("尝试在 " + path_way + " 中删除数据 " + id);

	fs.readFile(path_way, (err, data) => {
		if (err) return console.error("读取文件失败：", err);

		let thisJSONStr = data.toString();
		let thisJSON = JSON.parse(thisJSONStr);

		let temp = thisJSON.data.some((val, index) => {
			if (id === val.id) thisJSON.data.splice(index, 1);
			return id === val.id;
		});

		if (!temp) {
			console.log("找不到对象");
			return false;
		}

		thisJSON.total = thisJSON.data.length;
		thisJSONStr = JSON.stringify(thisJSON);
		fs.writeFile(path_way, thisJSONStr, (err) => {
			if (err) console.error("重写失败：", err);
			else console.log("删除成功");
		});
	});
};

let changeJson = (toChange, path_way) => {
	console.log("尝试在 " + path_way + " 中修改数据 " + JSON.stringify(toChange));
	fs.readFile(path_way, (err, data) => {
		if (err) return console.error("读取文件失败：", err);
		let thisJSONStr = data.toString();
		let thisJSON = JSON.parse(thisJSONStr);

		let temp = thisJSON.data.some((val) => {
			if (toChange.id === val.id) {
				for (let key in toChange) {
					//if (val[key] !== undefined) {
					val[key] = toChange[key];
					//}
				}
			}
			return toChange.id == val.id;
		});

		if (!temp) {
			thisJSON.data.push(toChange);
			console.log("找不到对象，追加成功");
		} else console.log("修改完成");

		thisJSON.data.sort(compUser);
		console.log("排序完成");

		thisJSON.total = thisJSON.data.length;
		thisJSONStr = JSON.stringify(thisJSON);
		writeQueue.push({ path_way, thisJSONStr }, writeQueueCallBack);
		writing = true;
		console.log("已加入文件写队列");
	});
};

let queryJSON = (id, path_way) => {
	console.log("尝试在 " + path_way + " 中查询数据 " + id);

	let data = fs.readFileSync(path_way);
	if (data === null) return console.error("读取失败");
	let thisJSONStr = data.toString();
	let thisJSON = JSON.parse(thisJSONStr);

	for (let val of thisJSON.data) {
		if (id === val.id) {
			console.log("找到对象 " + JSON.stringify(val));
			return val;
		}
	}

	console.log("找不到对象");
	return null;
};

let queryJSONAll = (path_way) => {
	console.log("尝试在 " + path_way + " 中查询所有数据");

	let data = fs.readFileSync(path_way);
	if (data === null) return console.error("读取失败");
	let thisJSONStr = data.toString();
	let thisJSON = JSON.parse(thisJSONStr);

	console.log("找到对象");
	return thisJSON.data;
};

let writeQueue = async.queue(({ path_way, thisJSONStr }, callBack) => {
	fs.writeFileSync(path_way, thisJSONStr);
	callBack(null, writeQueue.length());
}, 1);

writeQueue.drain(() => {
	writing = false;
	console.log("所有文件写操作执行完毕！");
});

let writeQueueCallBack = (err, rest) => {
	if (err) console.error("文件写队列发生错误", err);
	console.log("文件写队列剩余任务数量：" + rest);
};

//主要事件定义----------------------------------------------------------------

client.on("message", (e) => {
	//console.log(e);
	if (e.atme === false && e.raw_message.match(/jiou/) === null) return;

	//e.reply("已收到消息", true); //true表示引用对方的消息

	console.log("收到触发消息，raw内容为：\n" + e.raw_message);

	console.log("查询频繁队列：" + e.sender.user_id + " --- " + userTiming.get(e.sender.user_id));
	
	if (userTiming.get(e.sender.user_id) === undefined) userTiming.set(e.sender.user_id, 5);

	if (userTiming.get(e.sender.user_id) === 0) {
		e.reply("操作太快了，请休息一下", false);
		return;
	}

	userTiming.set(e.sender.user_id, userTiming.get(e.sender.user_id) - 1);
	setTimeout(() => {
		userTiming.set(e.sender.user_id, userTiming.get(e.sender.user_id) + 1);
	}, 60000);

	//查询帮助
	if (e.raw_message.match(/help/) !== null) {
		e.reply(HELPTEXT, false);
		return;
	}

	//默认
	e.reply("已收到消息，使用 jiou help 触发帮助", true);
});

//setInterval();

client.on("notice.group.increase", (e) => {
	console.log("触发进群事件");
	e.group.sendMsg("侦测到账号 " + e.user_id.toString() + " 进群");
});

client.on("notice.group.decrease", (e) => {
	console.log("触发退群事件");
	e.group.sendMsg("侦测到账号 " + e.user_id.toString() + " 退群");
});
