/*****************************************************************
 * 版权所有 (C) Lave.Zhang@outlook.com 2012
 * 本源代码仅供学习研究之用，不得用于商业目的。
 ****************************************************************/
using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Windows.Forms;

namespace MetroGraphApp
{
    public partial class FrmMain : Form
    {
        public FrmMain()
        {
            InitializeComponent();

            //打开地铁线路图
            metroGraphView1.OpenFromFile(Application.StartupPath + "\\MetroGraph.xml");
        }
    }
}