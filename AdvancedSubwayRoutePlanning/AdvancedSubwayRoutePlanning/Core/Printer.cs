using System.IO;
using System.Text;
using System.Collections.Generic;
namespace Core
{
    public class Printer
    {
        private StreamWriter output;

        public Printer(Stream os)
        {
            this.output = new StreamWriter(os, Encoding.Default);
        }

        public void PrintDirections(List<Connection> route)
        {
            if (route == null || route.Count == 0)
            {
                output.WriteLine("0");
                output.Flush();
                return;
            }
            output.WriteLine(route.Count + 1);
            string ls = route[0].LineName;
            route.ForEach(x =>
            {
                output.Write(x.BeginStation);
                if (x.LineName != ls)
                {
                    output.Write(" 换乘" + x.LineName);
                    ls = x.LineName;
                }
                output.WriteLine();
            });
            output.WriteLine(route.FindLast(x => true).EndStation.Name);
            output.Flush();
        }

        public void PrintSubwayLine(List<Station> line)
        {
            if (line == null || line.Count == 0)
            {
                output.WriteLine("没有此条线路！");
                output.Flush();
                return;
            }
            line.ForEach(x => output.WriteLine(x));
            output.Flush();
        }

        public void WriteLine(string s)
        {
            output.WriteLine(s);
            output.Flush();
        }
    }
}