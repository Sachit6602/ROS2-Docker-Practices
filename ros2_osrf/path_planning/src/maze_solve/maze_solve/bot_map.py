import cv2
import numpy as np

draw_intrstpts = False

class bot_mapper():

    def __init__(self):

        # State Variables
        self.graphified = False

        # Cropping control for removing maze boundary
        self.crp_amt = 5
        

    @staticmethod
    def get_surround_pixel_intensities(maze, curr_row, curr_col):

        maze = cv2.threshold(maze, 1, 1, cv2.THRESH_BINARY)[1]

        rows = maze.shape[0]
        cols = maze.shape[1]

        top_row = False
        btm_row = False
        lft_col = False
        rgt_col = False

        # Checking if there is a boundary condition
        if (curr_row==0):
            # Top row => Row above not accesible
            top_row = True
        if (curr_row == (rows-1)):
            # Bottom row ==> Row below not accesible
            btm_row = True
        if (curr_col == 0):
            # Left col ==> Col to the left not accesible
            lft_col = True
        if (curr_col == (cols-1)):
            # Right col ==> Col to the right not accesible
            rgt_col = True

        # Extracting surround pixel intensities and Addressing boundary conditions (if present)
        if (top_row or lft_col):
            top_left = 0
        else:
            top_left = maze[curr_row-1][curr_col-1]
        if( top_row or rgt_col ):
            top_rgt = 0
        else:
            top_rgt = maze[curr_row-1][curr_col+1]

        if( btm_row or lft_col ):
            btm_left = 0
        else:
            btm_left = maze[curr_row+1][curr_col-1]

        if( btm_row or rgt_col ):
            btm_rgt = 0
        else:
            btm_rgt = maze[curr_row+1][curr_col+1]
        
        # If the point we are at is anywhere on the top row, Then
        #             ===> Its top pixel is definitely not accesible
        if (top_row):
            top = 0
        else:
            top = maze[curr_row-1][curr_col]
        if (rgt_col):
            rgt = 0
        else:
            rgt = maze[curr_row][curr_col+1]
        
        if (btm_row):
            btm = 0
        else:
            btm = maze[curr_row+1][curr_col]

        if (lft_col):
            lft = 0
        else:
            lft = maze[curr_row][curr_col-1]

        no_of_pathways = ( top_left + top      + top_rgt  +
                           lft      + 0        + rgt      + 
                           btm_left + btm      + btm_rgt        
                         )
        if no_of_pathways>2:  
            print("  [ top_left , top      , top_rgt  ,lft    , rgt      , btm_left , btm      , btm_rgt   ] \n [ ",str(top_left)," , ",str(top)," , ",str(top_rgt)," ,\n   ",str(lft)," , ","-"," , ",str(rgt)," ,\n   ",str(btm_left)," , ",str(btm)," , ",str(btm_rgt)," ] ")
            print("\nno_of_pathways [row,col]= [ ",curr_row," , ",curr_col," ] ",no_of_pathways) 

        return top_left,top,top_rgt,rgt,btm_rgt,btm,btm_left,lft,no_of_pathways


    def one_pass(self,maze):
        maze_bgr = cv2.cvtColor(maze, cv2.COLOR_GRAY2BGR)

        #Creating a window to display Detected Interest Points
        cv2.namedWindow("Maze (Interest Points)",cv2.WINDOW_FREERATIO)
        rows = maze.shape[0]
        cols = maze.shape[1]

        for row in range(rows):
            for col in range(cols):
                if (maze[row][col]==255):
                    #Probabale Interest Point
                    top_left,top,top_rgt,rgt,btm_rgt,btm,btm_left,lft, paths = self.get_surround_pixel_intensities(maze.copy(),row,col)

                    if ( (row==0) or (row == (rows-1)) or (col==0) or (col == (cols-1)) ):
                        if (row == 0):
                            # Start
                            maze_bgr[row][col] = (0,128,255)
                            cv2.imshow("Maze (Interest Points)",maze_bgr)
                        
                        else:
                            # End (MAze Exit)
                            maze_bgr[row][col] = (0,255,0)
                            cv2.imshow("Maze (Interest Points)",maze_bgr)  
                    # Check if it is a (Dead End)        
                    elif (paths==1):
                        crop = maze[row-1:row+2,col-1:col+2]
                        print(" ** [Dead End] ** \n" ,crop)
                        maze_bgr[row][col] = (0,0,255)# Red color
                        if draw_intrstpts:
                            maze_bgr= cv2.circle(maze_bgr, (col,row), 10, (0,0,255),4)                       
                        cv2.imshow("Maze (Interest Points)",maze_bgr)
                    # Check if it is either a *Turn* or just an ordinary path
                    elif (paths==2):
                        crop = maze[row-1:row+2,col-1:col+2]
                        nzero_loc = np.nonzero(crop > 0)
                        nzero_ptA = (nzero_loc[0][0],nzero_loc[1][0])
                        nzero_ptB = (nzero_loc[0][2],nzero_loc[1][2])
                        if not ( ( (2 - nzero_ptA[0])==nzero_ptB[0] ) and 
                                    ( (2 - nzero_ptA[1])==nzero_ptB[1] )     ):
                            maze_bgr[row][col] = (255,0,0)
                            #if draw_intrstpts:
                                #maze_bgr= cv2.circle(maze_bgr, (col,row), 10, (255,0,0),2)                            
                            cv2.imshow("Maze (Interest Points)",maze_bgr)

                     # Check if it is either a *3-Junc* or a *4-Junc*
                    elif (paths>2):
                        if (paths ==3):
                            maze_bgr[row][col] = (255,244,128)
                            if draw_intrstpts:
                                maze_bgr = self.triangle(maze_bgr, (col,row), 10,(144,140,255),4)
                            
                            cv2.imshow("Maze (Interest Points)",maze_bgr)

                        else:
                            maze_bgr[row][col] = (128,0,128)
                            if draw_intrstpts:
                                cv2.rectangle(maze_bgr,(col-10,row-10) , (col+10,row+10), (255,215,0),4)
                            cv2.imshow("Maze (Interest Points)",maze_bgr)
                        
                          
                        


    def graphify(self,extracted_maze):

        # Check graph extracted or not from the maze
        if not self.graphified:
            cv2.imshow("Extracted_Maze [MazeConverter]", extracted_maze)

            # peforming thinning on maze to reduce area to paths that car could follow.
            thinned = cv2.ximgproc.thinning(extracted_maze)
            cv2.imshow("Maze (thinned)", thinned)

            # dilate and Perform thining again to minimize unneccesary interest point (i.e:turns)
            kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(2,2))
            thinned_dilated = cv2.morphologyEx(thinned, cv2.MORPH_DILATE, kernel)
            _, bw2 = cv2.threshold(thinned_dilated, 0, 255, cv2.THRESH_BINARY | cv2.THRESH_OTSU)        
            thinned = cv2.ximgproc.thinning(bw2)
            cv2.imshow("Maze (thinned*2)", thinned)

            
            #crop out Boundary that is not part of maze
            thinned_cropped = thinned[self.crp_amt:thinned.shape[0]-self.crp_amt,
                                      self.crp_amt:thinned.shape[1]-self.crp_amt]
        
            cv2.imshow("Maze (thinned*2)(Cropped)", thinned_cropped)

             #overlay found path on Maze Occupency Grid.
            extracted_maze_cropped = extracted_maze[self.crp_amt:extracted_maze.shape[0]-self.crp_amt,
                                                    self.crp_amt:extracted_maze.shape[1]-self.crp_amt]
            extracted_maze_cropped = cv2.cvtColor(extracted_maze_cropped, cv2.COLOR_GRAY2BGR)
            extracted_maze_cropped[thinned_cropped>0] = (0,255,255)
            cv2.imshow("Maze (thinned*2)(Cropped)(Path_Overlayed)", extracted_maze_cropped)
            
            # identify Interest Points in the path to further reduce processing time
            self.one_pass(thinned_cropped)
            #cv2.waitKey(0)
            self.maze = thinned_cropped
            self.graphified = True

