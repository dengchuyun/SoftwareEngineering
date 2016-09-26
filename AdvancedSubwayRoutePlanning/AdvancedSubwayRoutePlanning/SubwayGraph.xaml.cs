using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Data;
using System.Windows.Documents;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using System.Windows.Navigation;
using System.Windows.Shapes;
using Core;

namespace AdvancedSubwayRoutePlanning
{
    /// <summary>
    /// SubwayGraph.xaml 的交互逻辑
    /// </summary>
    public partial class SubwayGraph : UserControl
    {
        private SubwayMap subwayMap;

        public SubwayGraph()
        {
            InitializeComponent();
            subwayMap = BackgroundCore.SubwayMap;
        }

        protected override void OnRender(DrawingContext dc)
        {
            base.OnRender(dc);

            //绘制线路列表
            DrawLineList(dc);

            //绘制地铁线路
            //DrawSubwayGraph(dc);
        }

        private void DrawLineList(DrawingContext dc)
        {
            //遮罩层
            Rect rc = new Rect(5, 5, 140, (subwayMap.SubwayLines.Count + 1) * 15);
            dc.DrawRectangle(new SolidColorBrush(Color.FromArgb(180, 245, 245, 245)), new Pen(Brushes.Black, 0.5), rc);

            //线路列表
            double y = rc.Y + 15;
            foreach (SubwayLine line in subwayMap.SubwayLines)
            {
                //绘制线路标示线
                dc.DrawLine(new Pen(new SolidColorBrush(hexToColor(line.Color)), 5), new Point(rc.X + 10, y), new Point(rc.X + 70, y));

                //绘制线路名
                FormattedText formattedText = createFormattedText(line.Name);
                dc.DrawText(formattedText, new Point(rc.X + 80, y - formattedText.Height / 2));

                y += 15;
            }
        }

        private void DrawSubwayGraph(DrawingContext dc)
        {
            //绘制地铁路径
            foreach (Connection connection in subwayMap.Connections)
            {
                DrawConnection(dc, connection);
            }

            //绘制地铁站点
            foreach (Station station in subwayMap.Stations)
            {
                DrawStation(dc, station);
            }
        }

        private void DrawConnection(DrawingContext dc, Connection connection)
        {
            Point pt1 = new Point(connection.beginStation.X, connection.beginStation.Y);
            Point pt2 = new Point(connection.endStation.X, connection.endStation.Y);

            Pen pen = new Pen(new SolidColorBrush((hexToColor(((SubwayLine)subwayMap.SubwayLines.Find((SubwayLine line) => line.Name == connection.LineName)).Color))), 5);
            pen.LineJoin = PenLineJoin.Round;
        }

        private void DrawStation(DrawingContext dc, Station station)
        {

        }

        private FormattedText createFormattedText(string text)
        {
            return new FormattedText(text, new System.Globalization.CultureInfo(0x0804), FlowDirection.LeftToRight, new Typeface("Verdana"), 12, Brushes.Black);
        }

        private Color hexToColor(string color)
        {
            return (Color)ColorConverter.ConvertFromString(color);
        }
    }
}
