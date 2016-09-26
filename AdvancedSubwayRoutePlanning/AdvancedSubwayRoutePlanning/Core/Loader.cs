using System;
using System.IO;
using System.Xml;

namespace Core
{
    class Loader
    {
        public SubwayMap SubwayMap { get; private set; }
        public Loader()
        {
            SubwayMap = new SubwayMap();
        }
        public SubwayMap LoadFromXMLFile(string path)
        {
            if (!File.Exists(path))
            {
                throw new FileNotFoundException("文件不存在！");
            }
            if (new FileInfo(path).Extension.ToLowerInvariant() != ".xml")
            {
                throw new FormatException("文件类型错误！");
            }
            
            XmlDocument doc = new XmlDocument();
            doc.Load(path);
            XmlNodeList lines = doc.DocumentElement.ChildNodes;

            foreach (XmlNode eachline in lines)
            {
                string lineName = eachline.Attributes.GetNamedItem("name").InnerXml;
                string lineColor = eachline.Attributes.GetNamedItem("color").InnerXml;
                SubwayMap.AddSubwayLine(lineName, lineColor);
                XmlNodeList stations = eachline.SelectSingleNode("stations").ChildNodes;
                foreach (XmlNode sta in stations)
                {
                    XmlAttributeCollection aa = sta.Attributes;
                    SubwayMap.AddStation(aa.GetNamedItem("name").InnerXml, Convert.ToDouble(aa.GetNamedItem("x").InnerXml), Convert.ToDouble(aa.GetNamedItem("y").InnerXml));
                }
                XmlNodeList connections = eachline.SelectSingleNode("connections").ChildNodes;
                foreach (XmlNode con in connections)
                {
                    XmlAttributeCollection aa = con.Attributes;
                    SubwayMap.AddConnection(aa.GetNamedItem("begin").InnerXml, aa.GetNamedItem("end").InnerXml, lineName);
                    if (aa.GetNamedItem("type").InnerXml == "double")
                    {
                        SubwayMap.AddConnection(aa.GetNamedItem("end").InnerXml, aa.GetNamedItem("begin").InnerXml, lineName);
                    }
                }
            }
            return this.SubwayMap;
        }
    }
}
