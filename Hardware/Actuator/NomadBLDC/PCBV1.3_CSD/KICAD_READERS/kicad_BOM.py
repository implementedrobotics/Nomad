#!/Applications/Kicad/kicad.app/Contents/Frameworks/Python.framework/Versions/Current/bin/python

#
# note: for kicad mac users, you need to use the path specified by kicad to get this to run

from __future__ import print_function

import sys
import svgwrite
import kicad_netlist_reader
import pcbnew
import csv
import re

def myEqu(self, other):
    """myEqu is a more advanced equivalence function for components which is
       used by component grouping. Normal operation is to group components based
        on their value and footprint.

        In this example of a custom equivalency operator we compare the
        value, the part name and the footprint.
        """
    result = True
    if self.getValue() != other.getValue():
        result = False
    elif self.getPartName() != other.getPartName():
        result = False
    elif self.getFootprint() != other.getFootprint():
        result = False

    return result

class WriteCentroid:
    def __init__(self, net, board):
        self.board = board
        self.net = net
        self.SCALE = 1000000.0


    def createCentroid(self, centroid_file):
        try:
            f = open(centroid_file, 'w')
        except IOError:
            e = "Can't open output file for writing: " + centroid_file
            print( __file__, ":", e, sys.stderr )
            f = sys.stdout

        # Create a new csv writer object to use as the output formatter
        out = csv.writer( f, lineterminator='\n', delimiter=',', quotechar='\"', quoting=csv.QUOTE_ALL )
        # Output a set of rows as a header providing general information
        self.writerow( out, ['Source', self.net.getSource()] )
        self.writerow( out, ['Date:', self.net.getDate()] )
        self.writerow( out, ['Tool:', self.net.getTool()] )
        self.writerow( out, ['Generator:', sys.argv[0]] )
        self.writerow( out, ['TOP OF BOARD ONLY:'])
        
        columns = ['REF DES', 'VALUE', 'PACKAGE', 'FOOTPRINT', 'X LOC', 'Y LOC', 'ROTATION']

        self.writerow( out, [] )
        self.writerow( out, columns)

        components = self.net.getInterestingComponents()
        d = {}
        for c in components:
            ref = c.getRef()
            d[ref] = c

        for module in board.GetModules():
            rot = module.GetOrientation()/10.0,

            ref = module.GetReference()

            fpid = module.GetFPID()
            footprint = '{}:{}'.format(fpid.GetLibNickname(), fpid.GetLibItemName())
            footprint = re.sub(r'.*:','', footprint )
            # kill: TQFP-6410x10mmP0.5mm
            footprint = re.sub(r'TQFP-64.*','TQFP-64', footprint)
            # kill TDFN-8-1EP3x2mmP0.5mmEP1.80x1.65mm
            footprint = re.sub(r'TDFN-8.*','TDFN-8', footprint)
            # kill DFN-6-1EP3x3mmP1mmEP1.5x2.4mm
            footprint = re.sub(r'DFN-6.*','DFN-6', footprint)
            footprint = re.sub(r'_[0-9]*Metric','', footprint)
            footprint = re.sub(r'_[0-9]*Metric','', footprint)

            (pos_x, pos_y) = module.GetCenter()

            # columns = ['REF DES', 'VALUE', 'PACKAGE', 'FOOTPRINT', 'X LOC', 'Y LOC', 'ROTATION']

            if ( d[ref].getField('POPULATE') == '1'):
                row = []
                row.append( module.GetReference())
                row.append( d[ref].getValue() )
                row.append( footprint )
                row.append( footprint )
                row.append("{:4.3f}mm".format(pos_x / self.SCALE))
                row.append("{:4.3f}mm".format(pos_y / self.SCALE))
                row.append( "{:4.2f}".format(module.GetOrientation()/10.0) )
                self.writerow( out, row )

        
        f.close()

    def writerow(self, acsvwriter, columns ):
        utf8row = []
        for col in columns:
            utf8row.append( str(col) )  # currently, no change
        acsvwriter.writerow( utf8row )


class WriteBOM:
    def __init__(self, net, board):
        self.board = board
        self.net = net
        self.SCALE = 1000000.0
        self.boardbbox = self.board.ComputeBoundingBox()
        self.boardwidth = self.boardbbox.GetWidth() / self.SCALE
        self.boardheight = self.boardbbox.GetHeight() / self.SCALE

    def createBOM(self, bom_file):

        try:
            f = open(bom_file, 'w')
        except IOError:
            e = "Can't open output file for writing: " + bom_file
            print( __file__, ":", e, sys.stderr )
            f = sys.stdout

        # Override the component equivalence operator - it is important to do this
        # before loading the netlist, otherwise all components will have the original
        # equivalency operator.
        kicad_netlist_reader.comp.__eq__ = myEqu

        # subset the components to those wanted in the BOM, controlled
        # by <configure> block in kicad_netlist_reader.py
        components = self.net.getInterestingComponents()
        compfields = self.net.gatherComponentFieldUnion(components)
        partfields = self.net.gatherLibPartFieldUnion()

        # remove Reference, Value, Datasheet, and Footprint
        partfields -= set( ['Reference', 'Value', 'Datasheet', 'Footprint'] )
        columnset = compfields | partfields     # union

        # prepend an initial 'hard coded' list and put the enchillada into list 'columns'
        columns = ['Item', 'Qty', 'Reference(s)', 'Value', 'LibPart', 'LINK', 'P/N', 'MFN'] + sorted(list(columnset))
        columns = ['Item #', '*Ref Des', '*Qty', 'Manufacturer', '*Mfg Part #', 'Description / Value', '*Package', 'Type', 'Your Instructions / Notes']

        # Create a new csv writer object to use as the output formatter
        out = csv.writer( f, lineterminator='\n', delimiter=',', quotechar='\"', quoting=csv.QUOTE_ALL )

        # Output a set of rows as a header providing general information
        self.writerow( out, ['Source', self.net.getSource()] )
        self.writerow( out, ['Date:', self.net.getDate()] )
        self.writerow( out, ['Tool:', self.net.getTool()] )
        self.writerow( out, ['Generator:', sys.argv[0]] )
        self.writerow( out, ['Component Count:', len(components)] )
        self.writerow( out, ['width (mm):', ('%0.2lf' % (self.boardbbox.GetWidth() / 1000000.0))] )
        self.writerow( out, ['height (mm):', ('%0.2lf' % (self.boardbbox.GetHeight() / 1000000.0))] )
        # Get all of the components in groups of matching parts + values
        # (see kicad_netlist_reader.py)
        grouped = self.net.groupComponents(components)

        # Output all the interesting components individually first:
        item = 0
        for group in grouped:
            if (self.net.getGroupField(group, 'POPULATE') == '1'):
                item += 1

        self.writerow( out, ['Collated Components:', item] )

        self.writerow( out, [] )
        self.writerow( out, columns)

        # Output component information organized by group, aka as collated:
        row = []
        item = 0
        for group in grouped:
            del row[:]
            refs = ""

            # Add the reference of every component in the group and keep a reference
            # to the component so that the other data can be filled in once per group
            for component in group:
                if len(refs) > 0:
                    refs += ", "
                refs += component.getRef()
                c = component

            if (self.net.getGroupField(group, 'POPULATE') == '1'):
                item += 1
                # Item #
                row.append( item )
                # *Ref Des
                row.append( refs );
                # *Qty
                row.append( len(group) )
                # Manufacturer
                row.append( self.net.getGroupField(group, 'MANUFACTURER') );
                # *Mfg Part #
                row.append( self.net.getGroupField(group, 'MPN') );
                # Description / Value
                row.append( c.getValue() )
                # Package
                # Type
                # Your Instructions / Notes

                footprint = re.sub(r'.*:','', self.net.getGroupFootprint(group))
                # kill: TQFP-6410x10mmP0.5mm
                footprint = re.sub(r'TQFP-64.*','TQFP-64', footprint)
                # kill TDFN-8-1EP3x2mmP0.5mmEP1.80x1.65mm
                footprint = re.sub(r'TDFN-8.*','TDFN-8', footprint)
                # kill DFN-6-1EP3x3mmP1mmEP1.5x2.4mm
                footprint = re.sub(r'DFN-6.*','DFN-6', footprint)
                footprint = re.sub(r'_[0-9]*Metric','', footprint)
                footprint = re.sub(r'_[0-9]*Metric','', footprint)

                row.append( re.sub(r'_','', footprint) )
                # row.append( self.net.getGroupDatasheet(group) )
                row.append( 'SMD' )
                self.writerow( out, row  )

        f.close()

    def writerow(self, acsvwriter, columns ):
        utf8row = []
        for col in columns:
            utf8row.append( str(col) )  # currently, no change
        acsvwriter.writerow( utf8row )

if __name__ == '__main__':
    if len(sys.argv) != 2:
        print("Usage ", __file__, "<generic_netlist.xml>", file=sys.stderr)

    # lots of hardcoded path/files. Sue me. 

    pcb_file = '/Users/YOURNAME/Documents/kicad/PCB_V1.3_CSD/PCB_V1.3_CSD.kicad_pcb'

    print(sys.argv[1])

    BOM_name = "/Users/YOURNAME/Documents/bom_out.csv"
    centroid_name = "/Users/YOURNAME/Documents/centroid.csv"

    net = kicad_netlist_reader.netlist(sys.argv[1])
    pcb_file = net.getSource()
    pcb_file = re.sub('.sch','.kicad_pcb', pcb_file )
    print(pcb_file)

    board = pcbnew.LoadBoard(pcb_file)
    
    w = WriteBOM(net, board)
    w.createBOM(BOM_name)

    print(centroid_name)

    w = WriteCentroid(net, board)
    w.createCentroid(centroid_name)
