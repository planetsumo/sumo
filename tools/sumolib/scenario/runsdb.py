import sqlite3, os



class RunsDB:
  def __init__(self):
    self.conn = None
    self.cursor = None

  def buildDB(self, dbName="results.db", deletePrior=True):
    if deletePrior and os.path.exists(dbName):
      os.remove(dbName)
    self.conn = sqlite3.connect(dbName)
    self.cursor = self.conn.cursor()
    self.cursor.execute('CREATE TABLE run (id integer, key text, value text)')
    self.cursor.execute('CREATE TABLE result (runID integer, interval integer, key text, value real)')
    self.conn.commit()
    self.run = 0 # !!! if an old db is opened to be extended, this should be read from the db
    
  def addRun(self, scenario, kvDesc):
    self.run = self.run + 1
    cid = self.run - 1    
    for k in kvDesc:
      self.cursor.execute("INSERT INTO run VALUES (?,?,?)", (cid, k, kvDesc[k]))
    return cid

  def addResult(self, runID, interval, key, value):
    self.cursor.execute("INSERT INTO result VALUES (?,?,?,?)", (runID, interval, key, value))
    self.conn.commit()
    
  def close(self):
    self.conn.close()
    

