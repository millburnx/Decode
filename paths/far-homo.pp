{
  "startPoint": {
    "x": 89,
    "y": 8.4,
    "heading": "linear",
    "startDeg": 0,
    "endDeg": 180,
    "locked": false
  },
  "lines": [
    {
      "id": "line-649s4snu9z",
      "name": "Fire Preload",
      "endPoint": {
        "x": 89,
        "y": 13.6,
        "heading": "linear",
        "startDeg": 0,
        "endDeg": 0
      },
      "controlPoints": [],
      "color": "#88C75C",
      "locked": false,
      "waitBeforeMs": 0,
      "waitAfterMs": 0,
      "waitBeforeName": "",
      "waitAfterName": ""
    },
    {
      "id": "mml6izlv-ckhw6l",
      "endPoint": {
        "x": 133,
        "y": 9,
        "heading": "linear",
        "reverse": false,
        "startDeg": 0,
        "endDeg": 0
      },
      "controlPoints": [],
      "color": "#CBBD9C",
      "name": "Intake HP",
      "waitBeforeMs": 0,
      "waitAfterMs": 0,
      "waitBeforeName": "",
      "waitAfterName": ""
    },
    {
      "id": "mml513g2-0knh5s",
      "name": "Shoot HP",
      "endPoint": {
        "x": 89,
        "y": 13.6,
        "heading": "linear",
        "reverse": false,
        "startDeg": 0,
        "endDeg": 0
      },
      "controlPoints": [],
      "color": "#D76D8C",
      "waitBeforeMs": 0,
      "waitAfterMs": 0,
      "waitBeforeName": "",
      "waitAfterName": ""
    },
    {
      "id": "mmqunznn-lsfel4",
      "endPoint": {
        "x": 133,
        "y": 12,
        "heading": "tangential",
        "reverse": false,
        "startDeg": 0,
        "endDeg": 0
      },
      "controlPoints": [],
      "color": "#BBB969",
      "name": "Intake Flow 1",
      "waitBeforeMs": 0,
      "waitAfterMs": 0,
      "waitBeforeName": "",
      "waitAfterName": ""
    },
    {
      "id": "mmquomx8-a8k44l",
      "endPoint": {
        "x": 88,
        "y": 13.6,
        "heading": "tangential",
        "reverse": true
      },
      "controlPoints": [],
      "color": "#9CAA5B",
      "name": "Shoot Flow 1",
      "waitBeforeMs": 0,
      "waitAfterMs": 0,
      "waitBeforeName": "",
      "waitAfterName": ""
    },
    {
      "id": "mml5e167-kfs4i9",
      "name": "Park",
      "endPoint": {
        "x": 108,
        "y": 13.6,
        "heading": "linear",
        "reverse": false,
        "startDeg": 0,
        "endDeg": 0
      },
      "controlPoints": [],
      "color": "#7CCD99",
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
      "lineId": "line-649s4snu9z"
    },
    {
      "kind": "path",
      "lineId": "mml6izlv-ckhw6l"
    },
    {
      "kind": "path",
      "lineId": "mml513g2-0knh5s"
    },
    {
      "kind": "path",
      "lineId": "mmqunznn-lsfel4"
    },
    {
      "kind": "path",
      "lineId": "mmquomx8-a8k44l"
    },
    {
      "kind": "path",
      "lineId": "mml5e167-kfs4i9"
    }
  ],
  "settings": {
    "xVelocity": 75,
    "yVelocity": 58,
    "aVelocity": 3.141592653589793,
    "kFriction": 0.1,
    "rWidth": 14,
    "rHeight": 16.8,
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
    "onionNextPointOnly": false,
    "showHeadingArrow": false,
    "headingArrowLength": 50,
    "headingArrowColor": "#ffffff",
    "headingArrowThickness": 2,
    "pathOpacity": 1
  },
  "version": "1.2.1",
  "timestamp": "2026-05-30T23:28:51.628Z"
}