namespace Core
{
    class Station
    {
        private static int ID = 0;
        public int Id { get; }
        public string Name { get; }
        public double X { get; }
        public double Y { get; }
        public Station(string name, double x, double y)
        {
            this.Name = name;
            this.Id = ++ID;
            this.X = x;
            this.Y = y;
        }
        public override bool Equals(object obj)
        {
            if (obj == null || GetType() != obj.GetType())
            {
                return false;
            }
            return this.Id == ((Station)obj).Id;
        }
        public override int GetHashCode()
        {
            return this.Id;
        }
        public override string ToString()
        {
            return this.Name;
        }
    }
}
