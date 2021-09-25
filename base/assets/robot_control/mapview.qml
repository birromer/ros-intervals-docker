import QtQuick 2.0
import QtQuick.Controls 2.2
import QtLocation 5.3
import QtPositioning 5.0

Item {
    id: root
    signal sendCoordinates(real lat, real lon, bool isLeft)
    signal sendCoordinatesDoubleClick(real lat, real lon, real zoomLevel, bool forward)

    Plugin
    {
        id: mapPlugin
        locales: "fr_FR"
        name: "osm" // OpenStreetMap
    }

    Map
    {
        id: map
        anchors.fill: parent
        plugin: mapPlugin
        objectName: 'map'
        center: QtPositioning.coordinate(48.40157318, -4.519289017)
        zoomLevel: 14
    }

    MouseArea
    {
        id: mouse
        anchors.fill: map
        hoverEnabled: true
        acceptedButtons: Qt.LeftButton | Qt.RightButton
        property var coordinate: map.toCoordinate(Qt.point(mouseX, mouseY))
        onClicked: root.sendCoordinates(coordinate.latitude,coordinate.longitude,mouse.button & Qt.LeftButton)
        onDoubleClicked: root.sendCoordinatesDoubleClick(coordinate.latitude,coordinate.longitude,map.zoomLevel,mouse.button & Qt.LeftButton) 
    }
    
    Component
    {
        id: circle
        MapCircle
        {
            radius: 10.0
            color: 'green'
            border.width: 1
        }
    }

    Component
    {
        id: line
        MapPolyline
        {
            line.width: 3
            line.color: 'green'
            path: []
        }
    }

    function createCirle(position,size,name,color)
    {
        var o  = circle.createObject(map)
        o.center = position
        o.radius = size
        o.objectName = name
        o.color = color
        map.addMapItem(o)
        return o
    }

    function createLine(wp1,wp2,size,name,color)
    {
        var o  = line.createObject(map)
        o.addCoordinate(wp1);
        o.addCoordinate(wp2);
        o.line.width = size
        o.line.color = color
        o.objectName = name
        map.addMapItem(o)
        return o
    }

    Connections
    {
        target: rbctrl
        onDrawCircleSignal: 
        {
            var o = createCirle(coordinates,size,name,color)
        }
        onDrawLineSignal: 
        {
            var o = createLine(coordinates1,coordinates2,3,name,color);
        }
    }

}
