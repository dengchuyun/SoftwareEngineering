namespace Core
{
    public class Station
    {
        private static int ID = 0;
        public int Id { get; }
        public string Name { get; }
        public double X { get; }
        public double Y { get; }
        public bool IsTransfer { get; }

        public Station(string name, double x, double y,bool isTransfer)
        {
            this.Id = ++ID;
            this.Name = name;
            this.X = x;
            this.Y = y;
            this.IsTransfer = isTransfer;
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
