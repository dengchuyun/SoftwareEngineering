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

namespace AdvancedSubwayRoutePlanning
{
    /// <summary>
    /// SubwayGraph.xaml 的交互逻辑
    /// </summary>
    public partial class SubwayGraph : UserControl
    {
        public SubwayGraph()
        {
            InitializeComponent();
        }

        protected override void OnRender(DrawingContext dc)
        {
            base.OnRender(dc);

            DrawLineList(dc);
        }

        private void DrawLineList(DrawingContext dc)
        {
            Rect rc = new Rect(10, 10, 150, 13 * 15);
            dc.DrawRectangle(new SolidColorBrush(Color.FromArgb(180, 245, 245, 245)), new Pen(Brushes.Black, 0.5), rc);
        }
    }
}
