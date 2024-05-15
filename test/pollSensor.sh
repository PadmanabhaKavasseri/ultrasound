#!/bin/bash                                                                                                                                     
                                                                                                                                                
i=0                                                                                                                                             
                                                                                                                                                
while true                                                                                                                                      
do                                                                                                                                              
  I="$i"                                                                                                                                        
  PATH="/usr/chirp_${I}.csv"                                                                                                                    
  /usr/local/bin/tdk-chx01-get-data-app -l ${PATH} -d 5                                                                                            
  let i++                                                                                                                                       
done