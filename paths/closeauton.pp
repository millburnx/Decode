{
  "startPoint": {
    "x": 15,
    "y": 110,
    "heading": "linear",
    "startDeg": 180,
    "endDeg": 180,
    "locked": false
  },
  "lines": [
    {
      "id": "line-cct1zcwas24",
      "name": "Path 1",
      "endPoint": {
        "x": 48,
        "y": 96,
        "heading": "constant",
        "startDeg": 90,
        "endDeg": 180,
        "degrees": 180
      },
      "controlPoints": [],
      "color": "#7C7969",
      "locked": false,
      "waitBeforeMs": 0,
      "waitAfterMs": 0,
      "waitBeforeName": "",
      "waitAfterName": ""
    },
    {
      "id": "mk7ffrjg-k2x3qr",
      "name": "Path 2",
      "endPoint": {
        "x": 48,
        "y": 84,
        "heading": "constant",
        "reverse": false,
        "degrees": 180
      },
      "controlPoints": [],
      "color": "#C8BB77",
      "waitBeforeMs": 0,
      "waitAfterMs": 0,
      "waitBeforeName": "",
      "waitAfterName": ""
    },
    {
      "id": "mk7fg4hg-hcxyzo",
      "name": "Path 3",
      "endPoint": {
        "x": 17,
        "y": 84,
        "heading": "constant",
        "reverse": false,
        "degrees": 180
      },
      "controlPoints": [],
      "color": "#DAD75D",
      "waitBeforeMs": 0,
      "waitAfterMs": 0,
      "waitBeforeName": "",
      "waitAfterName": ""
    },
    {
      "id": "mk7fgcx7-v07h13",
      "name": "Path 4",
      "endPoint": {
        "x": 60,
        "y": 84,
        "heading": "constant",
        "reverse": false,
        "degrees": 180
      },
      "controlPoints": [],
      "color": "#9DB96C",
      "waitBeforeMs": 0,
      "waitAfterMs": 0,
      "waitBeforeName": "",
      "waitAfterName": ""
    }
  ],
  "shapes": [
    {
      "id": "triangle-1",
      "name": "Red Goal",
      "vertices": [
        {
          "x": 144,
          "y": 70
        },
        {
          "x": 144,
          "y": 144
        },
        {
          "x": 120,
          "y": 144
        },
        {
          "x": 138,
          "y": 119
        },
        {
          "x": 138,
          "y": 70
        }
      ],
      "color": "#dc2626",
      "fillColor": "#ff6b6b"
    },
    {
      "id": "triangle-2",
      "name": "Blue Goal",
      "vertices": [
        {
          "x": 6,
          "y": 119
        },
        {
          "x": 25,
          "y": 144
        },
        {
          "x": 0,
          "y": 144
        },
        {
          "x": 0,
          "y": 70
        },
        {
          "x": 7,
          "y": 70
        }
      ],
      "color": "#2563eb",
      "fillColor": "#60a5fa"
    }
  ],
  "sequence": [
    {
      "kind": "path",
      "lineId": "line-cct1zcwas24"
    },
    {
      "kind": "path",
      "lineId": "mk7ffrjg-k2x3qr"
    },
    {
      "kind": "path",
      "lineId": "mk7fg4hg-hcxyzo"
    },
    {
      "kind": "path",
      "lineId": "mk7fgcx7-v07h13"
    }
  ],
  "settings": {
    "xVelocity": 60,
    "yVelocity": 48,
    "aVelocity": 3.141592653589793,
    "kFriction": 0.1,
    "rWidth": 18,
    "rHeight": 18,
    "safetyMargin": 1,
    "maxVelocity": 40,
    "maxAcceleration": 30,
    "maxDeceleration": 30,
    "fieldMap": "decode.webp",
    "robotImage": "/robot.png",
    "theme": "auto",
    "showGhostPaths": false,
    "showOnionLayers": false,
    "onionLayerSpacing": 3,
    "onionColor": "#dc2626",
    "onionNextPointOnly": false
  },
  "version": "1.2.1",
  "timestamp": "2026-01-09T22:09:21.124Z"
}
