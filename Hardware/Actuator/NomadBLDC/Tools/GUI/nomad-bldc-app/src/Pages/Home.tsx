import React from 'react';

import Table from '@mui/material/Table';
import TableBody from '@mui/material/TableBody';
import TableCell from '@mui/material/TableCell';
import TableContainer from '@mui/material/TableContainer';
import TableHead from '@mui/material/TableHead';
import TableRow from '@mui/material/TableRow';
import Paper from '@mui/material/Paper';
import { Avatar, Box, Button, Grid, Stack, Typography } from '@mui/material';
import { useNavigate } from 'react-router-dom';

import MoreVertIcon from '@mui/icons-material/MoreVert';

import IconButton from '@mui/material/IconButton';

function createData(name: string, id: number, status: string, voltage: number, current: number, connected: boolean) {
    return { name, id, status, voltage, current, connected };
}

const rows = [createData('KFE', 0x10, 'IDLE', 23.85, 0.45, true), createData('HFE', 0x11, 'FOC', 22.85, 0.35, true)];

export default function Home() {
    const navigate = useNavigate();

    return (
        <Box sx={{ display: 'flex', padding: '12px' }}>
            <Stack>
                <Grid container>
                    <Grid item lg={6}>
                        <Typography variant="h5">Available Devices</Typography>
                    </Grid>

                    <Grid item lg={6}>
                        <Grid item direction="row">
                            <Button variant="contained">Add</Button>
                            <Button variant="contained">Scan</Button>
                        </Grid>
                    </Grid>
                </Grid>

                <TableContainer>
                    <Table sx={{ minWidth: 850 }} aria-label="simple table">
                        <TableHead>
                            <TableRow>
                                <TableCell>Device Info</TableCell>
                                <TableCell align="right">Status</TableCell>
                                <TableCell align="right">Connected</TableCell>
                                <TableCell align="right">Actions</TableCell>
                            </TableRow>
                        </TableHead>
                        <TableBody>
                            {rows.map((row) => (
                                <TableRow
                                    key={row.name}
                                    hover
                                    sx={{ '&:last-child td, &:last-child th': { border: 0 } }}
                                >
                                    <TableCell
                                        component="th"
                                        scope="row"
                                        onClick={() => {
                                            navigate('settings');
                                        }}
                                        sx={{ cursor: 'pointer' }}
                                    >
                                        <Grid container>
                                            <Grid item alignContent="center" lg={3}>
                                                <Avatar alt={row.name} src="." />
                                            </Grid>
                                            <Grid item lg={9}>
                                                <Typography variant="subtitle1">
                                                    {row.name} (0x{row.id.toString(16)})
                                                </Typography>

                                                <Typography color="textSecondary" variant="body2">
                                                    CAN ID: {row.id}
                                                </Typography>
                                                <Typography color="textSecondary" variant="body2">
                                                    Voltage: {row.voltage} V
                                                </Typography>
                                                <Typography color="textSecondary" variant="body2">
                                                    Current: {row.voltage} A
                                                </Typography>
                                            </Grid>
                                        </Grid>
                                    </TableCell>
                                    <TableCell align="right">{row.status}</TableCell>
                                    <TableCell align="right">
                                        <IconButton
                                            onClick={() => {
                                                console.log('Show Menu');
                                            }}
                                        >
                                            <MoreVertIcon />
                                        </IconButton>
                                    </TableCell>
                                </TableRow>
                            ))}
                        </TableBody>
                    </Table>
                </TableContainer>
            </Stack>
        </Box>
    );
}
