
int limit(int val, int limits)
{
  if (val > limits)
    val = limits;
  if (val < -limits)
    val = -limits;
  return val;
}

int range(int val, int lower, int upper)
{
  if (val > upper)
    val = upper;
  if (val < lower)
    val = lower;
  return val;
}

int sign(int val)
{
  if (val > 0)
    return 1;
  if (val < 0)
    return -1;
  return 0;
}

int dead_band(int val, int db)
{
  if (val > db)
    val -= db;
  else if (val < -db)
    val += db;
  else
    val = 0;
  return val;
}
