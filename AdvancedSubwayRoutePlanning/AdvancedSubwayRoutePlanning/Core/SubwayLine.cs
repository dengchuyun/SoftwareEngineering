using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Core
{
    class SubwayLine
    {
        public string Name { get; }
        public string Color { get; }
        public SubwayLine(string name, string color)
        {
            this.Name = name;
            this.Color = color;
        }
    }
}
