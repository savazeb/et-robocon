# robocon-program

**プログラムの作り方**
* 　extensions.jsonのファイルを作る
```
{
	"recommendations": [
		"lego-education.ev3-micropython"
	],
	"unwantedRecommendations": [
		"ms-python.python"
	]
}
```
*  launch.jsonのファイルを作る
```
{
	"version": "0.2.0",
	"configurations": [
		{
			"name": "Download and Run",
			"type": "ev3devBrowser",
			"request": "launch",
			"program": "/home/robot/${workspaceRootFolderName}/${relativeFile}",
			"interactiveTerminal": false
		}
	]
}
```
*  settings.jsonのファイルを作る
```
{
	"files.eol": "\n",
	"debug.openDebug": "neverOpen",
	"python.linting.enabled": false,
	"python.languageServer": "None"
}
```
* 　.pyのファイルを作る
