using System;
using System.IO;
using System.Xml;
using System.Collections;

namespace Core
{
    class Loader
    {
        public Loader() { }

        public Hashtable LoadCityMap(string path)
        {
            if (!File.Exists(path))
            {
                throw new FileNotFoundException("文件不存在！");
            }
            if (new FileInfo(path).Extension.ToLowerInvariant() != ".xml")
            {
                throw new FormatException("文件类型错误！");
            }
            Hashtable CityMap = new Hashtable();
            XmlDocument doc = new XmlDocument();
            doc.Load(path);
            XmlNodeList cities = doc.DocumentElement.ChildNodes;
            foreach (XmlNode city in cities)
            {
                CityMap.Add(city.Attributes.GetNamedItem("name").InnerXml, city.Attributes.GetNamedItem("src").InnerXml);
            }

            return CityMap;
        }

        public SubwayMap LoadSubwayMap(string path)
        {
            if (!File.Exists(path))
            {
                throw new FileNotFoundException("文件不存在！");
            }
            if (new FileInfo(path).Extension.ToLowerInvariant() != ".xml")
            {
                throw new FormatException("文件类型错误！");
            }
            SubwayMap SubwayMap = new SubwayMap();
            XmlDocument doc = new XmlDocument();
            doc.Load(path);
            XmlNodeList lines = doc.DocumentElement.ChildNodes;

            foreach (XmlNode eachline in lines)
            {
                string lineName = eachline.Attributes.GetNamedItem("lid").InnerXml;
                string lineColor = eachline.Attributes.GetNamedItem("lc").InnerXml.Remove(0,2);
                string lineLabel = eachline.Attributes.GetNamedItem("lb").InnerXml;
                SubwayMap.AddSubwayLine(lineName, lineLabel, "#" + lineColor);
                XmlNodeList stations = eachline.ChildNodes;
                string lastName = "";
                foreach (XmlNode sta in stations)
                {
                    XmlAttributeCollection a = sta.Attributes;

                    if (a.GetNamedItem("iu") == null || a.GetNamedItem("iu").InnerXml == null || a.GetNamedItem("iu").InnerXml == "" || a.GetNamedItem("iu").InnerXml == "false") continue;
                    string stationName= a.GetNamedItem("sid").InnerXml;
                    if (stationName == null || stationName == "") continue;
                    double x = Convert.ToDouble(a.GetNamedItem("x").InnerXml);
                    double y = Convert.ToDouble(a.GetNamedItem("y").InnerXml);
                    bool isTransfer = Convert.ToBoolean(a.GetNamedItem("ex").InnerXml);
                    SubwayMap.AddStation(stationName, x, y, isTransfer);
                    if (lastName != "")
                    {
                        DoubleLineCheck(SubwayMap, lastName, stationName, lineName);
                    }
                    lastName = stationName;
                }
                if (eachline.Attributes.GetNamedItem("loop").InnerXml == "true")
                {
                    string stationName= stations[0].Attributes.GetNamedItem("sid").InnerXml;
                    DoubleLineCheck(SubwayMap, lastName, stationName, lineName);
                }
            }
            SubwayMap.SortStations();
            return SubwayMap;
        }

        private void DoubleLineCheck(SubwayMap subwayMap, string lastName, string stationName, string lineName)
        {
            if (subwayMap.Connections.Exists(w => w.BeginStation.Name == lastName && w.EndStation.Name == stationName && w.Type != -1))
            {
                subwayMap.Connections.Find(w => w.BeginStation.Name == lastName && w.EndStation.Name == stationName && w.Type != -1).Type = 1;
                subwayMap.AddConnection(lastName, stationName, lineName, 2);
                subwayMap.AddConnection(stationName, lastName, lineName, -1);
            }
            else if (subwayMap.Connections.Exists(w => w.BeginStation.Name == stationName && w.EndStation.Name == lastName && w.Type != -1))
            {
                subwayMap.Connections.Find(w => w.BeginStation.Name == stationName && w.EndStation.Name == lastName && w.Type != -1).Type = 1;
                subwayMap.AddConnection(stationName, lastName, lineName, 2);
                subwayMap.AddConnection(lastName, stationName, lineName, -1);
            }
            else
            {
                subwayMap.AddConnection(lastName, stationName, lineName, 0);
                subwayMap.AddConnection(stationName, lastName, lineName, -1);
            }
        }
    }
}
